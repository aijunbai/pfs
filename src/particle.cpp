/*
 * particle.cpp
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/poisson.hpp>

#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <queue>
#include <particle.h>
#include "util.h"
#include "action.h"
#include "tracker.h"
#include "murty.h"
#include "common.h"
#include "tracker.h"
#include "task.h"
#include "params.h"
#include "gaussian.h"

using namespace std;

const uint Particle::mMurtyThreshold = 3;
const double Particle::mProbThreshold = 1.0e-6;

STATISTIC::Ptr Particle::mComparedPairsStat[2];
STATISTIC::Ptr Particle::mPushedPairsStat[2];
STATISTIC::Ptr Particle::mAllPairsStat[2];
STATISTIC::Ptr Particle::mComputedPermutationsStat[2];
STATISTIC::Ptr Particle::mAllPermutationsStat[2];
STATISTIC::Ptr Particle::mComputedCombinationStat[2];
STATISTIC::Ptr Particle::mAllCombinationsStat[2];
STATISTIC::Ptr Particle::mComputedTermsStat[2];
STATISTIC::Ptr Particle::mAllTermsStat[2];
STATISTIC::Ptr Particle::mTimeUsageStat[2];

STATISTIC::Ptr Particle::mAssignmentApproximateError;
STATISTIC::Ptr Particle::mFalseMissingApproximateError;


Particle::Particle():
    mAssignments(make_shared<Assignments>(0.0))
{

}

Particle::Particle(const Particle &o):
    enable_shared_from_this(o),
    mHumans(o.mHumans),
    mAssignments(o.mAssignments)
{

}

void Particle::CopyFrom(const Particle &o)
{
  mAssignments = o.mAssignments;

  mHumans.clear();
  foreach_(const HumanState::Ptr &hs, o.mHumans) {
    mHumans.push_back(make_shared<HumanState>(*hs));
  }
}

Particle::~Particle()
{

}

void Particle::Predict(double duration)
{
  if (Params::ins().death_rate > 0.0 && !mHumans.empty()) {
    const int m = SimpleRNG::ins().GetPoisson(
        Params::ins().death_rate * mHumans.size() * duration);

    for (int i = 0; i < m; ++i) {
      HumanList::iterator it = mHumans.begin();
      advance(it, SimpleRNG::ins().GetRand() % mHumans.size());
      mHumans.erase(it);

      if (mHumans.empty()) {
        break;
      }
    }
  }

  foreach_(HumanState::Ptr &hs, mHumans) {
    hs->IntentionAwarePredict(duration);
  }
}

Particle::Ptr Particle::Refine(CachedObservation &obs, double duration)
{
  HashSet<uint> matched_o;
  HashSet<uint> matched_h;

  vector<Detection::Ptr> refine1;         //false detections to be removed
  vector<HumanState::Ptr> refine2;    //missing detections to be removed

  if (!mAssignments->empty()) {
    Assignment::Ptr assignment = mAssignments->propose();

    for (Assignment::iterator it = assignment->begin();
        it != assignment->end(); ++it) {
      matched_o.insert(it->first);
      matched_h.insert(mHumans.get(it->second)->mID);
    }
  }

  for (uint o = 0; o < obs.Detections().size(); ++o) {
    if (mAssignments->empty() || !matched_o.count(o)) {
      Detection::Ptr &det = obs.Detections()[o];

      if (SimpleRNG::ins().GetUniform() <
          Observation::ProposeProb(det->mConfidence)) {
        refine1.push_back(det);
      }
    }
  }

  if (Params::ins().refinement_rate > 0.0) {
    foreach_(HumanState::Ptr &hs, mHumans) {
      if (mAssignments->empty() || !matched_h.count(hs->mID)) {
        if (SimpleRNG::ins().GetUniform() < Params::ins().refinement_rate) {
          refine2.push_back(hs);
        }
      }
    }
  }

  if (!refine1.empty() || !refine2.empty()) {
    if (Params::ins().threads) {
      random_shuffle(refine1.begin(), refine1.end(), SimpleRNG::ins());
      random_shuffle(refine2.begin(), refine2.end(), SimpleRNG::ins());
    }
    else {
      random_shuffle(refine1.begin(), refine1.end());
      random_shuffle(refine2.begin(), refine2.end());
    }

    Particle::Ptr refined = make_shared<Particle>(*this);

    foreach_(HumanState::Ptr &hs, refine2) {
      HumanList::iterator it =
          find(refined->mHumans.begin(), refined->mHumans.end(), hs);
      assert(it != refined->mHumans.end());
      refined->mHumans.erase(it);
    }

    foreach_(Detection::Ptr &det, refine1) {
      refined->mHumans.push_back(make_shared<HumanState>(det));
    }

    refined->ComputeObservationProb(obs, duration);
    return refined;
  }

  return shared_from_this();
}

void Particle::Reset(CachedObservation &obs)
{
  mAssignments = make_shared<Assignments>(0.0);
  mProbCache.clear();

  for (uint o = 0; o < obs.Detections().size(); ++o) {
    int h = 0;

    foreach_(HumanState::Ptr &hs, mHumans) {
      mProbCache[o][h] = obs.ObservationLikelihood(o, *hs);

      h += 1;
    }
  }
}

Particle::Ptr Particle::Update(CachedObservation &obs, double duration)
{
  Predict(duration);
  ComputeObservationProb(obs, duration);

  Particle::Ptr refined = Refine(obs, duration);
  return refined->Likelihood() > Likelihood()? refined: shared_from_this();
}

void Particle::ComputeObservationProb(
    CachedObservation &obs, double duration)
{
  if (Params::ins().approximation_test) {
    ComputeObservationProbImp(obs, duration, false);
    double v1 = Likelihood();

    ComputeObservationProbImp(obs, duration, true);
    double v2 = Likelihood();

    double e = fabs(v2 - v1) / v1;
    STATISTIC::Add(mFalseMissingApproximateError, e);
  }
  else {
    ComputeObservationProbImp(obs, duration, true);
  }
}

#define EN_QUEUE(i, j) do { \
  if ((i) < o.size() && (j) < s.size() && !added[(i)][(j)]) { \
      queue.push( make_pair( \
          o[(i)]->first * s[(j)]->first, \
          make_pair((i), (j)) \
        )); \
      added[(i)][(j)] = true; \
      pushed_pairs += 1; \
    } \
  } while (0)

void Particle::ComputeObservationProbImp(
    CachedObservation &obs, double duration, bool pruning)
{
  Reset(obs);

  const int O = obs.Detections().size();
  const int S = mHumans.size();

  if (mHumans.empty()) {
    double prob = obs.AllFalseDetectionWeight(O, duration);
    Assignment::Ptr assignment = make_shared<Assignment>(1.0, prob);

    mAssignments->push_back(assignment);
    mAssignments->total_prob = prob;
    return;
  }

  if (obs.Detections().empty()) {
    double prob = obs.MissingDetectionProb(S, S, duration);
    Assignment::Ptr assignment = make_shared<Assignment>(1.0, prob);

    mAssignments->push_back(assignment);
    mAssignments->total_prob = prob;
    return;
  }

  const Combination::big_int_t all_combinations =
      Combination::Binomial(O + S, O);
  Combination::big_int_t computed_combinations = 0;

  Combination::big_int_t all_terms = 0;
  Combination::big_int_t computed_terms = 0;

  if (mAllTermsStat[pruning]) {
    for (int i = 0; i <= min(O, S); ++i) {
      all_terms +=
          Combination::Binomial(O, i) *
          Combination::Binomial(S, i) *
          Combination::Factorial(i);
    }
  }

  Observation::PossibilityList o;
  Observation::PossibilityList s;

  for (int F = max(0, O - S); F <= O; ++F) {
    int M = S - O + F;

    if (O == F || S == M) {
      assert(O == F && S == M);

      double prob1 = obs.MissingDetectionProb(S, M, duration);
      double prob2 = obs.AllFalseDetectionWeight(O, duration);

      Assignment::Ptr assignment = make_shared<Assignment>(1.0, prob1 * prob2);
      mAssignments->push_back(assignment);
      mAssignments->total_prob += assignment->prob();

      computed_combinations += 1;
      computed_terms += 1;

      continue;
    }

    Observation::PossibilityList::Ptr a =
        obs.GetObservationPossibilities(O, O - F, duration);
    Observation::PossibilityList::Ptr b =
        obs.GetStatePossibilities(S, S - M, duration);

    o.insert(o.end(), a->begin(), a->end());
    s.insert(s.end(), b->begin(), b->end());
  }

  assert(mAssignments->size() == 1);

  sort(o.begin(), o.end(), PtrGreater<Observation::Possibility>());
  sort(s.begin(), s.end(), PtrGreater<Observation::Possibility>());

  const Combination::big_int_t all_pairs = o.size() * s.size();
  Combination::big_int_t compared_pairs = 0;
  Combination::big_int_t pushed_pairs = 0;

  priority_queue<pair<double, pair<int, int> > > queue;
  HashMap<int, HashMap<int, bool> > added;

  EN_QUEUE(0, 0);

  pair<double, pair<uint, uint> > top = queue.top();
  queue.pop();

  do {
    compared_pairs += 1;

    double false_missing = top.first;

    Combination::Table<int> *mo = o[top.second.first]->second;
    Combination::Table<int> *mh = s[top.second.second]->second;

    if (mo->size() == mh->size()) {
      Permutations::Ptr perms = ComputeTruePositiveProb(*mo, *mh, pruning);
      sort(perms->begin(), perms->end(), PtrGreater<Permutation>());

      STATISTIC::Add(mComputedPermutationsStat[pruning], perms->size());

      foreach_(Permutation::Ptr &perm, *perms) {
        if (perm->prob() * false_missing > mAssignments->front()->prob()) {
          Assignment::Ptr assignment =
              make_shared<Assignment>(perm->prob(), false_missing);

          bool full = true;
          for (uint i = 0; i < perm->size(); ++i) {
            int row = mo->operator ()(i);
            int col = mh->operator ()(perm->at(i));
            double prob = mProbCache[row][col];

            if (prob > mProbThreshold) {
              assignment->operator [](row) = col;
            }
            else {
              full = false;
              break;
            }
          }

          if (full) {
            if (!Params::ins().assignment_sampling) {
              mAssignments->clear();
            }

            mAssignments->push_back(assignment);
          }
          else if (pruning) {
            break;
          }
        }
        else if (pruning) {
          break;
        }
      }

      mAssignments->total_prob += perms->total_prob * false_missing;
      computed_combinations += 1;
      computed_terms += perms->size();

      if (computed_terms > Params::ins().max_terms && pruning) {
        break;
      }
    }

    bool can_prune = top.first < Params::ins().false_missing_pruning;

    EN_QUEUE(top.second.first + 1, top.second.second);
    EN_QUEUE(top.second.first, top.second.second + 1);

    if (queue.empty()) {
      break;
    }

    top = queue.top();
    queue.pop();

    if (can_prune && top.first < false_missing && pruning) {
      break;
    }
  } while (1);

  STATISTIC::Add(mComparedPairsStat[pruning], compared_pairs);
  STATISTIC::Add(mPushedPairsStat[pruning], pushed_pairs);
  STATISTIC::Add(mAllPairsStat[pruning], all_pairs);

  STATISTIC::Add(mComputedCombinationStat[pruning], computed_combinations);
  STATISTIC::Add(mAllCombinationsStat[pruning], all_combinations);

  STATISTIC::Add(mComputedTermsStat[pruning], computed_terms);
  STATISTIC::Add(mAllTermsStat[pruning], all_terms);

  sort(mAssignments->begin(), mAssignments->end(), PtrGreater<Assignment>());
}

void Particle::EnumerateAllPermutations(
    Permutations &perms,
    Permutation &perm,
    Combination::Table<int> &obs,
    Combination::Table<int> &hs,
    int i)
{
  if (i == int(perm.size()) - 1) {
    perm.prob1 = 1.0;
    for (uint k = 0; k < perm.size(); ++k) {
      double prob = mProbCache[obs(k)][hs(perm[k])];
      perm.prob1 *= prob;
    }

    perms.push_back(make_shared<Permutation>(perm));
    perms.total_prob += perm.prob();
  }
  else {
    for (uint j = i; j < perm.size(); ++j) {
      swap(perm[i], perm[j]);
      EnumerateAllPermutations(perms, perm, obs, hs, i + 1);
      swap(perm[i], perm[j]);
    }
  }
}

Particle::Permutations::Ptr Particle::ComputeFullTruePositiveProb(
    Combination::Table<int> &obs,
    Combination::Table<int> &hs)
{
  Permutation::Ptr perm = make_shared<Permutation>(0.0, 1.0);

  for (uint i = 0; i < obs.size(); ++i) {
    perm->push_back(i);
  }

  Permutations::Ptr perms = make_shared<Permutations>(0.0);
  EnumerateAllPermutations(*perms, *perm, obs, hs);

  return perms;
}

Particle::Permutations::Ptr Particle::ComputeTruePositiveProb(
    Combination::Table<int> &obs,
    Combination::Table<int> &hs,
    bool pruning)
{
  if (Params::ins().approximation_test && pruning) {
    Permutations::Ptr p1 = ComputeTruePositiveProbImp(obs, hs, false);
    Permutations::Ptr p2 = ComputeTruePositiveProbImp(obs, hs, true);

    double e = fabs(p2->total_prob - p1->total_prob) / p1->total_prob;
    STATISTIC::Add(mAssignmentApproximateError, e);

    return pruning? p2: p1;
  }
  else {
    return ComputeTruePositiveProbImp(obs, hs, pruning);
  }
}

Particle::Permutations::Ptr Particle::ComputeTruePositiveProbImp(
    Combination::Table<int> &obs,
    Combination::Table<int> &hs,
    bool pruning)
{
  assert(obs.size() == hs.size());

  uint n = obs.size();

  STATISTIC::Add(mAllPermutationsStat[pruning], Combination::Factorial(n));

  if (!Params::ins().approximation_test) {
    for (uint i = 0; i < n; ++i) {
      bool all_zeros = true;

      for (uint j = 0; j < n; ++j) {
        if (mProbCache[obs(i)][hs(j)] > mProbThreshold) {
          all_zeros = false;
          break;
        }
      }

      if (all_zeros) {
        return make_shared<Permutations>(0.0);
      }
    }

    for (uint i = 0; i < n; ++i) {
      bool all_zeros = true;

      for (uint j = 0; j < n; ++j) {
        if (mProbCache[obs(j)][hs(i)] > mProbThreshold) {
          all_zeros = false;
          break;
        }
      }

      if (all_zeros) {
        return make_shared<Permutations>(0.0);
      }
    }
  }

  if (n < mMurtyThreshold || !pruning) {
    return ComputeFullTruePositiveProb(obs, hs);
  }
  else {
    return Murty::ins().Solve(mProbCache, obs, hs);
  }
}

void Particle::Log(RCGLogger::Ptr &logger, bool verbose, const char *label)
{
  foreach_(HumanState::Ptr &hs, mHumans) {
    hs->Log(logger, verbose, label);
  }
}

void Particle::PrintStatistics()
{
  if (Params::ins().approximation_test) {
    PrintStatistics(true);
    PrintStatistics(false);

    TerminalLogger::log() << endl;
    STATISTIC::Print(
        mAssignmentApproximateError,
        "assignment approximate error");

    STATISTIC::Print(
        mFalseMissingApproximateError,
        "false-missing approximate error");
    TerminalLogger::log() << endl;
  }
  else {
    PrintStatistics(true);
  }
}

void Particle::PrintStatistics(bool pruning)
{
  TerminalLogger::log() << endl;
  TerminalLogger::log() << "pruning=" << pruning << endl;

  STATISTIC::Print(
      mAllPermutationsStat[pruning],
      "all permutations");
  STATISTIC::Print(
      mComputedPermutationsStat[pruning],
      "computed permutations");
  TerminalLogger::log() << "#permutation pruning rate: "
      << 1.0 - mComputedPermutationsStat[pruning]->GetMean() /
      mAllPermutationsStat[pruning]->GetMean() << endl;

  TerminalLogger::log() << endl;
  STATISTIC::Print(
      mAllPairsStat[pruning], "all pairs");
  STATISTIC::Print(
      mPushedPairsStat[pruning], "pushed pairs");
  STATISTIC::Print(
      mComparedPairsStat[pruning], "compared pairs");
  TerminalLogger::log() << "#push-pair pruning rate: "
      << 1.0 - mPushedPairsStat[pruning]->GetMean() /
      mAllPairsStat[pruning]->GetMean() << endl;
  TerminalLogger::log() << "#compair-pair pruning rate: "
      << 1.0 - mComparedPairsStat[pruning]->GetMean() /
      mAllPairsStat[pruning]->GetMean() << endl;

  TerminalLogger::log() << endl;
  STATISTIC::Print(
      mAllCombinationsStat[pruning],
      "all combinations");
  STATISTIC::Print(
      mComputedCombinationStat[pruning],
      "computed combinations");
  TerminalLogger::log() << "#combination pruning rate: "
      << 1.0 - mComputedCombinationStat[pruning]->GetMean() /
      mAllCombinationsStat[pruning]->GetMean() << endl;

  TerminalLogger::log() << endl;
  STATISTIC::Print(
      mAllTermsStat[pruning], "all terms");
  STATISTIC::Print(
      mComputedTermsStat[pruning], "computed terms");
  TerminalLogger::log() << "#term pruning rate: "
      << 1.0 - mComputedTermsStat[pruning]->GetMean() /
      mAllTermsStat[pruning]->GetMean() << endl;

  TerminalLogger::log() << endl;
  STATISTIC::Print(
      mTimeUsageStat[pruning],
      "overall observation function computation time (ms)");
}

void Particle::CreateStatistics()
{
  if (Params::ins().approximation_test) {
    CreateStatistics(true);
    CreateStatistics(false);

    STATISTIC::Create(mAssignmentApproximateError);
    STATISTIC::Create(mFalseMissingApproximateError);
  }
  else {
    CreateStatistics(true);
  }
}

void Particle::CreateStatistics(bool pruning)
{
  STATISTIC::Create(mComparedPairsStat[pruning]);
  STATISTIC::Create(mPushedPairsStat[pruning]);
  STATISTIC::Create(mAllPairsStat[pruning]);
  STATISTIC::Create(mComputedPermutationsStat[pruning]);
  STATISTIC::Create(mAllPermutationsStat[pruning]);
  STATISTIC::Create(mComputedCombinationStat[pruning]);
  STATISTIC::Create(mAllCombinationsStat[pruning]);
  STATISTIC::Create(mComputedTermsStat[pruning]);
  STATISTIC::Create(mAllTermsStat[pruning]);
  STATISTIC::Create(mTimeUsageStat[pruning]);
}



