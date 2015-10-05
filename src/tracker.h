/*
 * human_tracker.h
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#ifndef HUMAN_TRACKER_H_
#define HUMAN_TRACKER_H_

#include <vector>
#include <algorithm>
#include <cmath>
#include <list>
#include <queue>
#include <set>
#include <sstream>

#include "particle.h"
#include "geometry.h"
#include "observation.h"
#include "logger.h"
#include "intention.h"

class Task;
class HumanTracker;

typedef std::vector<Particle::Ptr> Particles;
typedef std::vector<vector2d*> Kernels;

#ifdef  NDEBUG
typedef HashSet<HumanState*> HumanStatePool;
#else
struct HumanStatePtrCmp {
  bool operator() (const HumanState *a, const HumanState *b) const {
    assert(a && b);
    return a->mID < b->mID;
  }
};

typedef std::set<HumanState*, HumanStatePtrCmp> HumanStatePool;
#endif

class ColorPool {
private:
  ColorPool();

public:
  static ColorPool &ins();

  uint Get(uint id);
  void Put(uint color);

private:
  std::queue<uint> mColors;

  static const uint32_t PALETTE[];
  static const int PALETTE_SIZE;
};

class IdentifiedHuman
{
public:
  typedef shared_ptr<IdentifiedHuman> Ptr;
  struct Track {
    HumanState::Ptr state;
    double confidence;
    RobotPose robot;

    Track(
        HumanState::Ptr s, double c,
        const RobotPose &r):
      state(s), confidence(c), robot(r)  {

    }
  };

public:
  const uint mID; //unique id
  const uint mBornTime;

  shared_ptr<uint> mColorCode;
  HumanStatePool mStatePool;
  std::vector<Track> mTrajectory;
  IntentionDistribution mIntentionDistri;
  HumanIntention *mIntention; //the dominant intention
  Detection::Ptr mDetection;
  shared_ptr<HumanTracker> mIntentionTracker;
  HashMap<uint, Kernels> mKernelsGivenIntention;
  HashMap<uint, HashSet<HumanState::Ptr> > mParticlesGivenIntention;
  bool mApproaching;

  IdentifiedHuman();
  virtual ~IdentifiedHuman();

  HumanIntention *Intention() const {
    return mIntentionDistri.Sampling();
  }

  bool Approaching(const uint samples); //whether the person is approaching the robot

  QColor DisplayingColor();

  double Confidence() const {
    double conf = double(mStatePool.size()) / Params::ins().num_particles;

    assert(conf >= 0.0);
    assert(conf <= 1.0);

    return conf;
  }

  int Age();
  void CreateIntentionTracker(Task *task, int debug_level);

  HumanState *ExpectedState() {
    return mTrajectory.empty()? 0: mTrajectory.back().state.get();
  }

  void BuildKernerlsGivenIntention(double duration);
};

class HumanTracker //Intention-aware Multi-Human Tracker
{
public:
  HumanTracker(Task *task, bool root, int debug_level, const char *log_name);
  virtual ~HumanTracker();

  void Update(Observation::Ptr obs, double duration = -1.0);

  double ExpectedIdentityCount();
  double AverageHumanCount();

  std::vector<IdentifiedHuman::Ptr> &IdentifiedHumans() {
    return mIdentifiedHumans;
  }

private:
  void NaiveResample(shared_ptr<Particles> &from, shared_ptr<Particles> &to);
  void LowVarianceResample(shared_ptr<Particles> &from, shared_ptr<Particles> &to);
  void Normalize(shared_ptr<Particles> &particles);
  void AddNoise(shared_ptr<Particles> &particles);
  void AddNoise(Particle::Ptr &particle);
  bool AllEmpty();

  typedef HashMap<Kernels*, HashMap<int, HashMap<int, double> > > KernelDensityCache;

  double ComputeMotionWeight(Particle &particle, KernelDensityCache &cache);
  double ComputeProposalWeight(Particle &particle, KernelDensityCache &cache);

  double OverallDensityEstimation(
      Particle &query, shared_ptr<Particles> &support, KernelDensityCache &cache);
  double KernelDensityEstimation(
      const vector2d &query, Kernels *support, KernelDensityCache &cache);

  void ComputePosteriorPredictive(shared_ptr<Particles> &support);
  void BuildPositionKernels(shared_ptr<Particles> &support, std::vector<int> &selected);

  void LogRCG(double duration);
  void LogParticles(shared_ptr<Particles> &particles, const char *tag);
  void LogProposal(shared_ptr<Particles> &particles);
  void LogIdentifiedHuman(IdentifiedHuman::Ptr &human, double text_position);
  void LogIdentifiedHuman(IdentifiedHuman::Ptr &human);
  void LogAssignments(int pid, Particle::Ptr &particle);
  void LogAssignmentsRCG();

  std::pair<int, int> PositionToIndex(const vector2d &pos) {
    int i = rint(pos.x * 100.0 / 5.0);
    int j = rint(pos.y * 100.0 / 5.0);

    return std::make_pair(i, j);
  }

  void ParticleFiltering(double duration);
  void HumanIdentification(double duration);
  void IntentionRecgonition(double duration);
  void SensorResetting(double duration);

  void Resetting(double reset);
  void UpdateIdentifiedHumans();

  void HumanIdentificationImp(double duration);
  void BuildStatePools();

  void ClearGhosts(
      std::vector<IdentifiedHuman::Ptr> &identified_humans,
      HashMap<int, HashMap<int, double> > &hi_prob,
      std::vector<Particle::Ptr> &particles);
  void BuildIntentionDistribution(IdentifiedHuman::Ptr &);
  void UpdateIntentionDistribution(IdentifiedHuman::Ptr &ih, HumanState &hs);

  HumanIntention *GetDominantIntention(
      HumanIntention *, IntentionDistribution &intentions);

  void PublishIdentifiedHumans();

public:
  void UpdateRobotPose(RobotPose &pose);
  const RobotPose &GetRobotPose();

  static double TransformConfidence(double conf) {
    return 1.0 / (1.0 + exp(-2.0 * conf));
  }

  static uint NextID() {
    return mIDCounter++;
  }

public:
  bool InFieldOfView(const vector2d& v) const {
    double dir1 = mRobotPose.mAngle - RAD(Params::ins().view_width / 2);
    double dir2 = mRobotPose.mAngle + RAD(Params::ins().view_width / 2);
    double dir = (v - mRobotPose.mPosition).angle();

    return _angle::IsAngleRadInBetween(dir1, dir, dir2);
  }

  vector2d FalseDetection() {
    const vector2d &robot = GetRobotPose().mPosition;

    vector2d eov1 = robot +
        vector2d(Params::ins().view_length, 0.0).rotate(
            GetRobotPose().mAngle - RAD(Params::ins().view_width / 2));
    vector2d eov2 = robot +
        vector2d(Params::ins().view_length, 0.0).rotate(
            GetRobotPose().mAngle + RAD(Params::ins().view_width / 2));

    double r1 = SimpleRNG::ins().GetUniform();
    double r2 = SimpleRNG::ins().GetUniform();

    return (1 - sqrt(r1)) * robot + (sqrt(r1) * (1 - r2)) *
        eov1 + (sqrt(r1) * r2) * eov2;
  }

  Observation &CurrentObservation() {
    assert(!mObservationHistory.empty());
    return *mObservationHistory.back();
  }

  class Worker: public SimpleThread {
   public:
     typedef shared_ptr<Worker> Ptr;

     Worker(
         HumanTracker *tracker,
         int id = 0, int threads = 1):
       mTracker(tracker),
       mID(id),
       mThreads(threads)
     {

     }

   public:
     HumanTracker *mTracker;
     const uint mID;
     const uint mThreads;
   };

  class ParticleFilteringWorker: public Worker {
  public:
    typedef shared_ptr<ParticleFilteringWorker> Ptr;

    ParticleFilteringWorker(
        HumanTracker *tracker,
        double duration,
        int id = 0, int threads = 1):
          Worker(tracker, id, threads),
          mDuration(duration)
    {

    }

    void StartRoutine();

  private:
    double mDuration;
    CachedObservation::Ptr mObservation;
  };

  class HumanIdentificationWorker: public Worker {
  public:
    typedef shared_ptr<HumanIdentificationWorker> Ptr;

    HumanIdentificationWorker(
        HumanTracker *tracker,
        std::vector<IdentifiedHuman::Ptr> &identified_humans_,
        HashMap<int, HashMap<int, double> > &hi_prob,
        int id = 0, int threads = 1):
          Worker(tracker, id, threads),
          mConverged(true),
          mIdentifiedHumans(identified_humans_),
          mProbMat(hi_prob)
    {

    }

    void StartRoutine();

    bool mConverged;

  private:
    std::vector<IdentifiedHuman::Ptr> &mIdentifiedHumans;
    HashMap<int, HashMap<int, double> > &mProbMat;
  };

  class KernelDensityWorker: public Worker {
   public:
     typedef shared_ptr<KernelDensityWorker> Ptr;

     KernelDensityWorker(
         HumanTracker *tracker,
         int id = 0, int threads = 1):
           Worker(tracker, id, threads)
     {

     }

     void StartRoutine();

   private:
     KernelDensityCache mCache;
   };

public:
  enum {
    LOG_STATE = 0,
    LOG_OBS,
    LOG_STATE_OBS,
    LOG_ALL,

    LOG_MODE_SIZE
  };

  bool mRoot; //the root filter
  int mDebugLevel;
  shared_array<RCGLogger::Ptr> mRCGLogger;
  shared_ptr<std::ofstream> mTextLoggerFile;

public:
  Task *mTask;
  RobotPose mRobotPose;
  std::vector<RobotPose> mTrajectory;
  std::list<Observation::Ptr> mObservationHistory;

  shared_ptr<Particles> mParticles;
  shared_ptr<Particles> mRefinedParticles;

  std::vector<IdentifiedHuman::Ptr> mIdentifiedHumans;

  HashMap<Particles*, std::pair<double, double> > mNegativeBinomial; //r=alhpa, p=beta/(1+beta)
  HashMap<Particles*, Kernels> mPositionKernels;

  static const double mKernelBandwidth;
  static const double mKernelSize;
  static const double mIdentificationMinProb;
  static const double mSensorResttingThreshold;
  static const double mDominantIntentionRate;
  static const double mNegativeBinomialAlpha;
  static const double mNegativeBinomialBeta;

  STATISTIC::Ptr mObservationSizeStat;
  STATISTIC::Ptr mNonEmptyObservationSizeStat;
  STATISTIC::Ptr mAllKernelsStat;
  STATISTIC::Ptr mComputedKernelsStat;
  STATISTIC::Ptr mUpdateTimeIntervalStat;
  STATISTIC::Ptr mParticleFilteringTimeUsage;
  STATISTIC::Ptr mHumanIdentificationTimeUsage;
  STATISTIC::Ptr mIntentionRecognitionTimeUsage;
  STATISTIC::Ptr mRobotSpeedStat;
  STATISTIC::Ptr mIdentityCountStat;
  STATISTIC::Ptr mAverageHumanCountStat;
  STATISTIC::Ptr mExpectedIdentityCountStat;
  STATISTIC::Ptr mIdentifiedHumanConfidence;
  STATISTIC::Ptr mEMConvergedSteps;

public:
  static const double mBackgroundDensity;

  static uint mCurrentStep;
  static double mCumulativeDuration;
  static uint mIDCounter;

  StreamLogger mTextLogger;
};

#endif /* HUMAN_TRACKER_H_ */
