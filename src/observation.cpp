/*
 * observation.cpp
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#include <boost/math/distributions/poisson.hpp>
#include <boost/math/distributions/beta.hpp>
#include <observation.h>

#include "gaussian.h"
#include "particle.h"
#include "tracker.h"
#include "combination.h"

using namespace std;

const double Observation::true_positive_alpha = 2.0;
const double Observation::true_positive_beta = 1.0;
const double Observation::false_positive_alpha = 1.0;
const double Observation::false_positive_beta = 2.0;

Observation::Observation(): mAllFalseProb(1.0)
{

}

Observation::~Observation()
{
}

void Observation::Reset()
{
  mAllFalseProb = 1.0;

  foreach_(Detection::Ptr &det, mDetections) {
    mAllFalseProb *= FalsePositiveLikelihood(det->mConfidence);
  }

  mAllFalseProb = Max(mAllFalseProb, double(FLT_EPSILON));
}

double Observation::ObservationLikelihoodTimed(
    const TimedDetection &det, HumanState &hs)
{
  double duration = HumanTracker::mCumulativeDuration - det.second;
  duration = Max(duration, Params::ins().sleep_duration);

  vector2d pos = hs.mPosition - hs.mVelocity * duration;

  return ObservationLikelihoodSingle(det.first->mPosition, pos);

  return 0.0;
}

double Observation::ObservationLikelihoodAugmented(Detection &det, HumanState &hs)
{
  double prob = ObservationLikelihoodSingle(det.mPosition, hs.mPosition);

  if (hs.mDetection && prob > 0.0) {
    prob *= ObservationLikelihoodTimed(*hs.mDetection, hs);
  }

  return prob;
}

double Observation::ObservationLikelihoodSingle(
    const vector2d &obs, const vector2d &pos)
{
  static const double error = Params::ins().observation_error;

  double dist = (obs - pos).length();
  double prob = Gaussian::ins().pdf(0.0, error, dist);

  return prob;
}

double Observation::ObservationLikelihood(int o, HumanState &hs)
{
  Detection::Ptr det = mDetections[o];
  double prob = TruePositiveLikelihood(det->mConfidence);

  if (prob > 0.0) {
    if (Params::ins().velocity_augment) {
      prob *= ObservationLikelihoodAugmented(*det, hs);
    }
    else {
      prob *= ObservationLikelihoodSingle(det->mPosition, hs.mPosition);
    }
  }

  prob = Max(prob, Particle::mProbThreshold);

  return prob;
}

double Observation::FalseDetectionWeight(
    Combination::Table<int> &obs, double duration)
{
  const int O = mDetections.size();
  const int N = obs.size();
  const int F = mDetections.size() - N;

  double false_prob = 1.0;

  for (int o = 0; o < O; ++o) {
    bool found = false;

    for (int i = 0; i < N; ++i) {
      if (obs(i) == o) {
        found = true;
        break;
      }
    }

    if (!found) {
      false_prob *= FalsePositiveLikelihood(mDetections[o]->mConfidence);
    }
  }

  return false_prob * FalseDetectionProb(O, F, duration);
}

double Observation::AllFalseDetectionWeight(uint O, double duration)
{
  return mAllFalseProb * FalseDetectionProb(O, O, duration);
}

double Observation::MissingDetectionWeight(
    uint s, Combination::Table<int> &hs, double duration)
{
  return MissingDetectionProb(s, s - hs.size(), duration);
}

double Observation::FalseDetectionProb(
    uint o, uint f, double duration)
{
  if (o == 0) {
    return 1.0;
  }

  if (Params::ins().false_rate * duration <= 0.0) {
    return f == 0? 1.0: 0.0;
  }

  double prob1 = Combination::ins().Factorial(f) *
      pow(Params::ins().false_density, f);

  boost::math::poisson_distribution<> dist(
      Params::ins().false_rate * duration);
  double prob2 = boost::math::pdf(dist, f);

  return prob1 * prob2;
}

double Observation::MissingDetectionProb(
    uint s, uint m, double duration)
{
  if (s == 0) {
    return 1.0;
  }

  if (Params::ins().missing_rate * s * duration <= 0.0) {
    return m == 0? 1.0: 0.0;
  }

  double prob1 = 1.0 / Combination::ins().Binomial(s, m);

  boost::math::poisson_distribution<> dist(
      Params::ins().missing_rate * s * duration);
  double prob2 = boost::math::pdf(dist, m);

  return prob1 * prob2;
}

Observation::PossibilityList::Ptr Observation::GetObservationPossibilities(
    int O, int M, double duration)
{
  PossibilityList::Ptr o = make_shared<PossibilityList>();
  Combination::Table<Combination::Table<int> > &obs_set =
      Combination::ins()(O)(M);

  for (uint i = 0; i < obs_set.size(); ++i) {
    double prob = FalseDetectionWeight(obs_set(i), duration);
    o->push_back(make_shared<Possibility>(prob, &obs_set(i)));
  }

  return o;
}

Observation::PossibilityList::Ptr Observation::GetStatePossibilities(
    int S, int M, double duration)
{
  PossibilityList::Ptr s = make_shared<PossibilityList>();
  Combination::Table<Combination::Table<int> > &hs_set =
      Combination::ins()(S)(M);

  for (uint i = 0; i < hs_set.size(); ++i) {
    double prob = MissingDetectionWeight(S, hs_set(i), duration);
    s->push_back(make_shared<Possibility>(prob, &hs_set(i)));
  }

  return s;
}

double Observation::TruePositiveLikelihood(double conf)
{
  static boost::math::beta_distribution<> dist(
      true_positive_alpha, true_positive_beta);

  assert(conf >= 0.0);
  assert(conf <= 1.0);

  return boost::math::pdf(dist, conf) + HumanTracker::mBackgroundDensity;
}

double Observation::FalsePositiveLikelihood(double conf)
{
  static boost::math::beta_distribution<> dist(
      false_positive_alpha, false_positive_beta);

  assert(conf >= 0.0);
  assert(conf <= 1.0);

  return boost::math::pdf(dist, conf) + HumanTracker::mBackgroundDensity;
}

double Observation::ProposeProb(double conf)
{
  double a = TruePositiveLikelihood(conf);
  double b = FalsePositiveLikelihood(conf);

  return a / (a + b) + HumanTracker::mBackgroundDensity;
}

double Observation::SampleTrueConfidence()
{
  return SimpleRNG::ins().GetBeta(true_positive_alpha, true_positive_beta);
}

double Observation::SampleFalseConfidence()
{
  return SimpleRNG::ins().GetBeta(false_positive_alpha, false_positive_beta);
}


double CachedObservation::ObservationLikelihood(int o, HumanState &hs)
{
  if (mLikelihoodCache.count(o) && mLikelihoodCache[o].count(hs.mID)) {
    return mLikelihoodCache[o][hs.mID];
  }

  mLikelihoodCache[o][hs.mID] = mObservation.ObservationLikelihood(o, hs);
  return mLikelihoodCache[o][hs.mID];
}

double CachedObservation::FalseDetectionProb(uint O, uint F, double duration)
{
  if (mFalseWProbCache.count(O) && mFalseWProbCache[O].count(F)) {
    return mFalseWProbCache[O][F];
  }

  mFalseWProbCache[O][F] = mObservation.FalseDetectionProb(O, F, duration);
  return mFalseWProbCache[O][F];
}

double CachedObservation::MissingDetectionProb(uint S, uint M, double duration)
{
  if (mMissingProbCache.count(S) && mMissingProbCache[S].count(M)) {
    return mMissingProbCache[S][M];
  }

  mMissingProbCache[S][M] = mObservation.MissingDetectionProb(S, M, duration);
  return mMissingProbCache[S][M];
}

Observation::PossibilityList::Ptr CachedObservation::GetObservationPossibilities(
    int O, int M, double duration)
{
  if (mObservationCombinationsCache.count(O) && mObservationCombinationsCache[O].count(M)) {
    return mObservationCombinationsCache[O][M];
  }

  mObservationCombinationsCache[O][M] = mObservation.GetObservationPossibilities(O, M, duration);
  return mObservationCombinationsCache[O][M];
}

Observation::PossibilityList::Ptr CachedObservation::GetStatePossibilities(
    int S, int M, double duration)
{
  if (mStateCombinationsCache.count(S) && mStateCombinationsCache[S].count(M)) {
    return mStateCombinationsCache[S][M];
  }

  mStateCombinationsCache[S][M] = mObservation.GetStatePossibilities(S, M, duration);
  return mStateCombinationsCache[S][M];
}
