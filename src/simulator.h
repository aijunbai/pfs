/*
 * simulator.h
 *
 *  Created on: Feb 3, 2014
 *      Author: baj
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "logger.h"
#include "intention.h"
#include "observation.h"
#include "state.h"
#include "tracker.h"

class Task;
class HumanTracker;
class IdentifiedHuman;

class Simulator
{
public:
  struct Human: public HumanState {
  private:
    Human(const Human &o);

  public:
    typedef shared_ptr<Human> Ptr;

    Human(
        const vector2d &pos,
        const vector2d &target,
        HumanIntention *intention);

    void SetID(uint id) {
      const_cast<uint&>(mID) = id;
    }

    void Predict(double duration);

    double TargetAngle() {
      return (mTarget - Position()).angle();
    }

    double TargetDist() {
      return (mTarget - Position()).length();
    }

    vector2d mTarget;
    std::vector<vector2d> mTrajectory;
  };

public:
  Simulator(Task *task, HumanTracker *tracker);
  virtual ~Simulator();

  virtual void Initialize(double expected_humans) = 0;
  virtual void Simulate(double dt, double birth_rate, double death_rate) = 0;
  virtual bool End() = 0;
  virtual void CheckConsistence(Observation &obs) = 0;
  virtual void LogRCG() = 0;
  virtual bool InFieldOfView(const vector2d& v) = 0;

  virtual void GenerateObservations(Observation &obs, double duration);

  virtual void Display() {

  }

  void Run(int argc, char **argv);

private:
  struct EvaluationMetrics {
    typedef shared_ptr<EvaluationMetrics> Ptr;

    EvaluationMetrics(
        HumanTracker *tracker,
        Simulator *simulator,
        double distance_threshold,
        bool weighted,
        double confidence_threshold = 0.0):
          mTracker(tracker), mSimulator(simulator),
          distance_threshold(distance_threshold),
          weighted(weighted),
          confidence_threshold(MinMax(0.0, confidence_threshold, 1.0))
    {
      mD = 0.0;
      mC = 0.0;
      mR = 0.0;
      mI = 0.0;
      mM = 0.0;
      mFP = 0.0;
      mMME = 0.0;
      mG = 0.0;
      mMT = 0.0;
      mML = 0.0;
      mPT = 0.0;
      mFM = 0.0;
    }

    ~EvaluationMetrics();

    void Evaluate();
    void Complete();

    double MOTP() {
      return mD / mC;
    }

    double MHIP() {
      return mI / mC;
    }

    double MOTA() {
      return 1.0 - (mM + mFP + mMME) / mG;
    }

    double Recall() {
      return mC / mG;
    }

    double Precision() {
      return mC /mR;
    }

    void PrintCLEARValues();

    typedef std::pair<Human::Ptr, IdentifiedHuman::Ptr> Mapping;
    std::set<Mapping> mCLEARMapping;

    typedef std::pair<vector2d, shared_ptr<vector2d> > Track;
    typedef std::list<Track> Tracks;
    HashMap<uint, Tracks> mTracks;

    double mC; //mathced
    double mR; //reported
    double mD; //distances of mathced
    double mI; //intention presision of mathced
    double mM; //missing tracked
    double mFP; //false positive
    double mMME; //id switch
    double mG; //ground truth
    double mMT; //most tracked
    double mPT; //partially tracked
    double mML; //most lost
    double mFM; //fragmetation

    HumanTracker *mTracker;
    Simulator *mSimulator;

    const double distance_threshold;

    const bool weighted;
    const double confidence_threshold;
  };

public:
  Task *mTask;
  HumanTracker *mTracker;
  std::list<Human::Ptr> mHumans;

  HashSet<EvaluationMetrics::Ptr> mMetrics;
};

#endif /* SIMULATOR_H_ */
