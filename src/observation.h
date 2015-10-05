/*
 * observation.h
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#ifndef OBSERVATION_H_
#define OBSERVATION_H_

#include <vector>
#include <list>
#include "geometry.h"
#include "common.h"
#include "combination.h"
#include "statistic.h"

class HumanTracker;
class HumanState;

struct Detection {
  typedef shared_ptr<Detection> Ptr;

  vector2d mPosition;  //pos in ground
  double mOrientation; //estimated orientation, if any
  double mConfidence; //[0, 1], if any
  double mHeight[2];  //bounding box height in image & world frame, if available
  double mWidth[2];   //bounding box width in image & world frame, if available

  Detection(
      vector2d pos /*= vector2d(0.0, 0.0)*/,
      double orientation /*= -1.0*/,
      double conf /*= 0.0*/,
      double ih /*= -1.0*/,
      double iw /*= -1.0*/,
      double rh /*= -1.0*/,
      double rw /*= -1.0*/): mPosition(pos), mOrientation(orientation)
  {
    mHeight[0] = ih;
    mWidth[0] = iw;

    mHeight[1] = rh;
    mWidth[1] = rw;

    if (Params::ins().detection_confidence) {
      if (mHeight[1] > 0.0 && mWidth[1] > 0.0) {
        double area = mHeight[1] * mWidth[1];

        if (area < Params::ins().human_area_min ||
            area > Params::ins().human_area_max) {
          conf = 0.0;
        }
      }
    }
    else {
      conf = 0.5;
    }

    mConfidence = MinMax(0.0, conf, 1.0);
    if (isinf(mOrientation) || isnan(mOrientation)) {
      mOrientation = DBL_MAX;
    }
  }

  double x() const {
    return mPosition.x;
  }

  double y() const {
    return mPosition.y;
  }

  double &Orientation() {
    return mOrientation;
  }

  const double &Orientation() const {
    return mOrientation;
  }

  void SetOrientation(double orient) {
    mOrientation = orient;
  }
};

typedef std::pair<Detection::Ptr, double> TimedDetection;

class Observation
{
public:
  typedef shared_ptr<Observation> Ptr;

public:
  Observation();
  virtual ~Observation();

  double ObservationLikelihood(int o, const HumanState &hs);
  double FalseDetectionProb(uint O, uint F, double duration);
  double MissingDetectionProb(uint S, uint M, double duration);

  double FalseDetectionWeight(Combination::Table<int> &obs, double duration);
  double MissingDetectionWeight(uint S, Combination::Table<int> &hs, double duration);

  double AllFalseDetectionWeight(uint O, double duration);

  static double TruePositiveLikelihood(double conf);
  static double FalsePositiveLikelihood(double conf);
  static double ProposeProb(double conf);

  static double SampleTrueConfidence();
  static double SampleFalseConfidence();

  void Reset();

private:
  double ObservationLikelihoodSingle(const Detection &det, const HumanState &hs);
  double ObservationLikelihoodAugmented(const Detection &det, const HumanState &hs);
  double ObservationLikelihoodTimed(const TimedDetection &det, const HumanState &hs);

public:
  std::vector<Detection::Ptr> mDetections;

private:
  double mAllFalseProb;

public:
  struct Possibility: public std::pair<double, Combination::Table<int>* > {
    typedef shared_ptr<Possibility> Ptr;

    Possibility(const double &prob, Combination::Table<int>* const tbl):
      std::pair<double, Combination::Table<int>* >(prob, tbl)
      {

      }
  };

  struct PossibilityList: public std::vector<Possibility::Ptr> {
    typedef shared_ptr<PossibilityList> Ptr;
  };

  PossibilityList::Ptr GetObservationPossibilities(
      int O, int M, double duration);
  PossibilityList::Ptr GetStatePossibilities(
      int S, int M, double duration);

private:
  static const double true_positive_alpha;
  static const double true_positive_beta;
  static const double false_positive_alpha;
  static const double false_positive_beta;
};

class CachedObservation
{
public:
  typedef shared_ptr<CachedObservation> Ptr;

  CachedObservation(Observation &obs): mObservation(obs) {

  }

  double FalseDetectionProb(uint O, uint F, double duration);
  double MissingDetectionProb(uint S, uint M, double duration);
  double ObservationLikelihood(int o, HumanState &hs);
  Observation::PossibilityList::Ptr GetObservationPossibilities(
      int O, int M, double duration);
  Observation::PossibilityList::Ptr GetStatePossibilities(
      int S, int M, double duration);

  std::vector<Detection::Ptr> &Detections() {
    return mObservation.mDetections;
  }

  double AllFalseDetectionWeight(uint O, double duration) {
    return mObservation.AllFalseDetectionWeight(O, duration);
  }

private:
  Observation &mObservation;

  HashMap<int, HashMap<int, double> > mLikelihoodCache;
  HashMap<int, HashMap<int, double> > mFalseWProbCache;
  HashMap<int, HashMap<int, double> > mMissingProbCache;
  HashMap<int, HashMap<int, Observation::PossibilityList::Ptr> > mObservationCombinationsCache;
  HashMap<int, HashMap<int, Observation::PossibilityList::Ptr> > mStateCombinationsCache;
};


#endif /* OBSERVATION_H_ */
