/*
 * human_tracker.cpp
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#include <boost/math/distributions/negative_binomial.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/gamma.hpp>
#include <boost/random/poisson_distribution.hpp>
#include <ctime>
#include <algorithm>

#include "terminal_utils.h"
#include <tracker.h>
#include <input_stream.h>
#include "action.h"
#include "timer.h"
#include "task.h"
#include "params.h"
#include "gaussian.h"
#include "helpers.h"
#include "murty.h"

using namespace std;

const double HumanTracker::mBackgroundDensity = 1.0e-3;
const double HumanTracker::mKernelBandwidth = 0.5;
const double HumanTracker::mKernelSize = 3.0;
const double HumanTracker::mIdentificationMinProb = 1.0e-6;
const double HumanTracker::mSensorResttingThreshold = 0.25;
const double HumanTracker::mDominantIntentionRate = 0.7;
const double HumanTracker::mNegativeBinomialAlpha = 2.0;
const double HumanTracker::mNegativeBinomialBeta = 1.0;

uint HumanTracker::mCurrentStep = 0;
double HumanTracker::mCumulativeDuration = 0;
uint HumanTracker::mIDCounter = 0;

const uint32_t ColorPool::PALETTE[] =
{
 0xFF0000, 0x00FF00, 0x0000FF,
 0xFF8800, 0x00FF88, 0x8800FF,
 0xFF0088, 0x88FF00, 0x0088FF,
 0x880000, 0x008800, 0x000088,
 0x888800, 0x008888, 0x880088,
 0xFFFF00, 0x00FFFF, 0xFF00FF
};

const int ColorPool::PALETTE_SIZE = 18;

namespace {
string TextLoggerStamp()
{
  stringstream ss;
  ss << "HumanTracker @ "
      << HumanTracker::mCurrentStep << " -: ";

  return ss.str();
}
}

ColorPool::ColorPool()
{
  for (int i = 0; i < PALETTE_SIZE; ++i) {
    Put(PALETTE[i]);
  }
}

ColorPool &ColorPool::ins()
{
  static ColorPool color_pool;
  return color_pool;
}

uint ColorPool::Get(uint id)
{
  if (!mColors.empty()) {
    uint color = mColors.front();
    mColors.pop();

    return color;
  }
  else {
    TerminalLogger::log() << "No colors available!" << endl;

    uint hash = id * 2654435761;
    return PALETTE[hash % PALETTE_SIZE];
  }
}

void ColorPool::Put(uint color)
{
  mColors.push(color);
}

IdentifiedHuman::~IdentifiedHuman()
{
  if (mColorCode) {
    ColorPool::ins().Put(*mColorCode);
  }
}

bool IdentifiedHuman::Approaching(const uint samples)
{
//  typedef pair<ros::Time, vector2d> TimedPosition;
//
//  vector<TimedPosition> humanTimePosition;
//  double distThreshold = 2.5;
//  double timeThreshold = 0.8;
//  double percCorrect = 0.5;
//  double directionThreshold = -0.7;
//
//  foreach_r_(Track &track, mTrajectory) {
//    vector2d rel_pos = (track.state->Position() -
//        track.robot.mPosition).rotate(-track.robot.mAngle);
//    humanTimePosition.push_back(make_pair(track.time, rel_pos));
//
//    if (humanTimePosition.size() >= samples) {
//      break;
//    }
//  }
//
//  if (humanTimePosition.size() < samples) {
//    return false;
//  }
//
//  reverse(humanTimePosition.begin(), humanTimePosition.end());
//
//  vector<double> distToRobot;
//
//  foreach_(TimedPosition &tp, humanTimePosition) {
//    double dist = tp.second.length();
//    distToRobot.push_back(dist);
//  }
//
//  vector<vector2d> veloc;
//  vector<double> velocNorm;
//  vector<vector2d> velocDir;
//
//  for (int i = 0; i < int(humanTimePosition.size()) - 1; ++i) {
//    vector2d vel = (humanTimePosition[i+1].second - humanTimePosition[i].second)
//        / fabs((humanTimePosition[i+1].first - humanTimePosition[i].first).toSec());
//
//    veloc.push_back(vel);
//    velocNorm.push_back(vel.length());
//    velocDir.push_back(vel.norm());
//  }
//
//  if (distToRobot.back() < distThreshold
//      && (humanTimePosition.back().first
//          - humanTimePosition.front().first).toSec() > timeThreshold) {
//    double sum = 0.0;
//
//    for (uint i = 0; i < velocDir.size(); ++i) {
//      if (veloc[i].length() > 1.0e-6) {
//        sum += velocDir[i].x <= directionThreshold;
//      }
//    }
//
//    //PRINT_VALUE(sum);
//    if (sum >= percCorrect * velocDir.size()) {
//      return true;
//    }
//  }

  return false;
}

QColor IdentifiedHuman::DisplayingColor()
{
  if (!mColorCode) {
    mColorCode = make_shared<uint>(ColorPool::ins().Get(mID));
  }

  return QColor(*mColorCode);
}

int IdentifiedHuman::Age()
{
  return HumanTracker::mCurrentStep - mBornTime;
}

void IdentifiedHuman::CreateIntentionTracker(Task *task, int debug_level)
{
  std::stringstream ss;
  ss << "human-" << mID;

  mIntentionTracker = make_shared<HumanTracker>(
      task, false, debug_level, ss.str().c_str());
}

IdentifiedHuman::IdentifiedHuman():
  mID(HumanTracker::NextID()),
  mBornTime(HumanTracker::mCurrentStep),
  mIntention(0),
  mApproaching(false)
{

}

void IdentifiedHuman::BuildKernerlsGivenIntention(double duration)
{
  mParticlesGivenIntention.clear();
  mKernelsGivenIntention.clear();

//  for (uint i = 0; i < IntentionFactory::ins().Size(); ++i) {
//    foreach_(HumanState *hs, mStatePool) {
//      mParticlesGivenIntention[i].insert(make_shared<HumanState>(*hs));
//    }
//  }
//
//  for (uint i = 0; i < IntentionFactory::ins().Size(); ++i) {
//    foreach_(const HumanState::Ptr &hs, mParticlesGivenIntention[i]) {
//      hs->SetIntention(i);
//      hs->IntentionAwarePredict(duration, false);
//      mKernelsGivenIntention[i].push_back(&(hs->Position()));
//    }
//  }
}

HumanTracker::HumanTracker(
    Task *task,
    bool root,
    int debug_level,
    const char *log_name):
    mRoot(root),
    mDebugLevel(debug_level),
    mTask(task)
{
  if (mRoot && mDebugLevel > 0) {
    STATISTIC::Create(mObservationSizeStat);
    STATISTIC::Create(mNonEmptyObservationSizeStat);
    STATISTIC::Create(mAllKernelsStat);
    STATISTIC::Create(mComputedKernelsStat);
    STATISTIC::Create(mUpdateTimeIntervalStat);
    STATISTIC::Create(mParticleFilteringTimeUsage);
    STATISTIC::Create(mHumanIdentificationTimeUsage);
    STATISTIC::Create(mIntentionRecognitionTimeUsage);
    STATISTIC::Create(mRobotSpeedStat);
    STATISTIC::Create(mIdentityCountStat);
    STATISTIC::Create(mAverageHumanCountStat);
    STATISTIC::Create(mExpectedIdentityCountStat);
    STATISTIC::Create(mIdentifiedHumanConfidence);
    STATISTIC::Create(mEMConvergedSteps);

    Particle::CreateStatistics();
    Murty::CreateStatistics();
  }

  if (mRoot && mDebugLevel > 1) {
    mRCGLogger = shared_array<RCGLogger::Ptr>(
        new RCGLogger::Ptr[LOG_MODE_SIZE]);

    string path = DEFAULT_LOG_PATH;

    if (!mRoot && Params::ins().hierarchical_filters) {
      path += "intentions/";
    }

    path += log_name;

    mRCGLogger[LOG_STATE] = make_shared<RCGLogger>((path + "-state").c_str());
    mRCGLogger[LOG_OBS] = make_shared<RCGLogger>((path + "-obs").c_str());
    mRCGLogger[LOG_STATE_OBS] = make_shared<RCGLogger>((path + "-state-obs").c_str());
    mRCGLogger[LOG_ALL] = make_shared<RCGLogger>((path + "-all").c_str());

    if (mDebugLevel > 2) {
      path += ".log";
      mTextLoggerFile = make_shared<ofstream>(path.c_str());

      if (!mTextLoggerFile || !mTextLoggerFile->good()) {
        PRINT_ERROR("Error: open text log file " << path);
        exit(1);
      }

      mTextLogger.Open(*mTextLoggerFile, TextLoggerStamp, 0);
    }
  }

  mRobotPose.mPosition.x = 0.0;
  mRobotPose.mPosition.y = 0.0;
  mRobotPose.mAngle = 0.0;
}

HumanTracker::~HumanTracker()
{
  if (mTextLoggerFile) {
    mTextLoggerFile->close();
  }

  if (mRoot && mDebugLevel > 0) {
    STATISTIC::Print(
        mObservationSizeStat, "observation size");
    STATISTIC::Print(
        mNonEmptyObservationSizeStat, "non empty observation size");

    TerminalLogger::log() << endl;
    STATISTIC::Print(mAllKernelsStat, "all kernels");
    STATISTIC::Print(mComputedKernelsStat, "computed kernels");
    if (mComputedKernelsStat) {
      TerminalLogger::log() << "#kernel pruning rate: "
          << 1.0 - mComputedKernelsStat->GetMean()
          / mAllKernelsStat->GetMean() << endl;
    }

    TerminalLogger::log() << endl;

    Particle::PrintStatistics();
    Murty::PrintStatistics();

    STATISTIC::Print(mParticleFilteringTimeUsage, "particle filtering (ms)");
    STATISTIC::Print(mHumanIdentificationTimeUsage, "human identification (ms)");
    STATISTIC::Print(mIntentionRecognitionTimeUsage, "intention recognition (ms)");

    TerminalLogger::log() << endl;
    STATISTIC::Print(mUpdateTimeIntervalStat, "update time interval (s)");
    STATISTIC::Print(mRobotSpeedStat, "robot travelled distance per cycle");
    if (mRobotSpeedStat) {
      TerminalLogger::log() << "#total robot travelled distance: "
          << mRobotSpeedStat->GetTotal() << endl;
    }

    STATISTIC::Print(mAverageHumanCountStat,
                     "human in particle count");
    STATISTIC::Print(mIdentityCountStat,
                     "identified human count");
    STATISTIC::Print(mExpectedIdentityCountStat,
                     "expected identified human count");
    STATISTIC::Print(mIdentifiedHumanConfidence,
                     "identified human confidence");
    STATISTIC::Print(mEMConvergedSteps,
                     "EM converged steps");
  }
}

void HumanTracker::Resetting(double reset)
{
  foreach_(Particle::Ptr &particle, *mParticles) {
    if (SimpleRNG::ins().GetUniform() < reset) {
      particle->mHumans.clear();

      foreach_(Detection::Ptr &det, CurrentObservation().mDetections) {
        if (SimpleRNG::ins().GetUniform() <
            Observation::ProposeProb(det->mConfidence)) {
          particle->mHumans.push_back(make_shared<HumanState>(det));
        }
      }
    }
  }

  BuildStatePools();
}

void HumanTracker::SensorResetting(double duration)
{
  if (mIdentifiedHumans.size() > 1 && !CurrentObservation().mDetections.empty()) {
    double confidence =
        ExpectedIdentityCount() / mIdentifiedHumans.size();

    if (confidence < mSensorResttingThreshold) {
      double reset = 1.0 - confidence;

      if (mDebugLevel > 0) {
        TerminalLogger::log() << "Sensor resetting reset=" << reset << endl;
      }
      mTextLogger << "Sensor resetting reset=" << reset << endl;

      Resetting(reset);
    }
  }
}

void HumanTracker::Normalize(shared_ptr<Particles> &particles)
{
  double total_likelihood = 0.0;

  foreach_(Particle::Ptr &p, *particles) {
    total_likelihood += p->Likelihood();
  }

  foreach_(Particle::Ptr &p, *particles) {
    p->Likelihood() /= total_likelihood;
  }
}

void HumanTracker::AddNoise(Particle::Ptr &particle)
{
  foreach_(HumanState::Ptr &hs, particle->mHumans) {
    hs->AddNoise();
  }
}

void HumanTracker::AddNoise(shared_ptr<Particles> &particles)
{
  foreach_(Particle::Ptr &p, *particles) {
    AddNoise(p);
  }
}

bool HumanTracker::AllEmpty()
{
  foreach_(Particle::Ptr &p, *mParticles) {
    if (!p->mHumans.empty()) {
      return false;
    }
  }

  return true;
}

void HumanTracker::Update(Observation::Ptr obs, double duration)
{
  if (!mParticles || !mRefinedParticles) {
    mParticles = make_shared<Particles>(Params::ins().num_particles);
    mRefinedParticles = make_shared<Particles>(Params::ins().num_particles);

    for (uint i = 0; i < Params::ins().num_particles; ++i) {
      mParticles->at(i) = make_shared<Particle>();
    }
  }

  mPositionKernels.clear();
  mNegativeBinomial.clear();

  if (mRoot) {
    RobotPose pose = mRobotPose;
    InputStream::ins().Filter(pose, obs->mDetections, duration);
    UpdateRobotPose(pose);
  }
  else {
    assert(duration > 0.0);
  }

  mTrajectory.push_back(mRobotPose);

  if (mRoot && duration > 0.0) {
    mCumulativeDuration += duration;
  }

  vector<RobotPose>::iterator it = mTrajectory.begin();
  while (mTrajectory.size() > Params::ins().trajectory_size) {
    it = mTrajectory.erase(it);
  }

  if (mDebugLevel > 0) {
    if (mRoot) {
      TerminalLogger::log() << "Observed #human="
          << obs->mDetections.size() << endl;
      TerminalLogger::log() << "Duration=" << duration << endl;
    }

    mTextLogger << "Observed #human="
        << obs->mDetections.size() << endl;
    mTextLogger << "Duration=" << duration << endl;
  }

  STATISTIC::Add(mObservationSizeStat, obs->mDetections.size());
  if (obs->mDetections.size()) {
    STATISTIC::Add(mNonEmptyObservationSizeStat, obs->mDetections.size());
  }

  mObservationHistory.push_back(obs); //save into history

  if (AllEmpty()) {
    Resetting(1.0);
  }

  {
    list<Observation::Ptr>::iterator it = mObservationHistory.begin();
    while (mObservationHistory.size() > Params::ins().trajectory_size) {
      it = mObservationHistory.erase(it);
    }
  }

  if (duration > 0.0) {
    STATISTIC::Add(mUpdateTimeIntervalStat, duration);

    ParticleFiltering(duration);
    HumanIdentification(duration);
    IntentionRecgonition(duration);
    SensorResetting(duration);
  }

  LogRCG(duration);

  //clean up
  foreach_(Particle::Ptr &particle, *mParticles) {
    Particle::HumanList::iterator it = particle->mHumans.begin();
    while (it != particle->mHumans.end()) {
      if ((*it)->Age() != 0 && (*it)->mIdentity == 0) {
        it = particle->mHumans.erase(it);
        continue;
      }
      ++it;
    }
  }

  if (mDebugLevel > 0) {
    if (mRoot) {
      TerminalLogger::log() << "Average #human="
          << AverageHumanCount() << endl;
      TerminalLogger::log() << "Identified #human="
          << mIdentifiedHumans.size() << endl;
      TerminalLogger::log() << "Expected identified #human="
          << ExpectedIdentityCount() << endl;
      TerminalLogger::log() << endl;
    }

    mTextLogger << "Average #human="
        << AverageHumanCount() << endl;
    mTextLogger << "Identified #human="
        << mIdentifiedHumans.size() << endl;
    mTextLogger << "Expected identified #human="
        << ExpectedIdentityCount() << endl;
    mTextLogger << endl;
  }

  if (mRoot) {
    mCurrentStep += 1;
  }
}

void HumanTracker::NaiveResample(shared_ptr<Particles> &from, shared_ptr<Particles> &to)
{
  if (from->empty()) {
    return;
  }

  assert(from->size() == Params::ins().num_particles
      && to->size() == Params::ins().num_particles);

  HashSet<int> chosen;

  vector<double> cmf(Params::ins().num_particles + 1, 0.0);

  for (uint i = 0; i < Params::ins().num_particles; ++i) {
    cmf[i+1] = cmf[i] + (*from)[i]->Likelihood();
  }
  cmf[Params::ins().num_particles] = 1.0;

  for (uint m = 0; m < Params::ins().num_particles; ++m) {
    uint i = distance(cmf.begin(), upper_bound(
        cmf.begin(), cmf.end(), SimpleRNG::ins().GetUniform())) - 1;

    i %= Params::ins().num_particles;

    if (chosen.count(i)) {
      (*to)[m] = make_shared<Particle>();
      (*to)[m]->CopyFrom(*(*from)[i]);
    }
    else {
      chosen.insert(i);
      (*to)[m] = (*from)[i];
    }

    mTextLogger << "Resampling: " << m << "<-" << i << endl;
  }

  mTextLogger << endl;
}

void HumanTracker::LowVarianceResample(shared_ptr<Particles> &from, shared_ptr<Particles> &to)
{
  if (from->empty()) {
    return;
  }

  assert(from->size() == Params::ins().num_particles
      && to->size() == Params::ins().num_particles);

  HashSet<int> chosen;

  double r = SimpleRNG::ins().GetUniform() / Params::ins().num_particles;
  double c = (*from)[0]->Likelihood();

  int i = 0;
  for (uint m = 0; m < Params::ins().num_particles; ++m) {
    double u = r + double(m) / Params::ins().num_particles;

    while (u > c) {
      i = (i + 1) % Params::ins().num_particles;
      c = c + (*from)[i]->Likelihood();
    }

    if (chosen.count(i)) {
      (*to)[m] = make_shared<Particle>();
      (*to)[m]->CopyFrom(*(*from)[i]);
    }
    else {
      chosen.insert(i);
      (*to)[m] = (*from)[i];
    }

    mTextLogger << "Resampling: " << m << "<-" << i << endl;
  }

  mTextLogger << endl;
}

const RobotPose &HumanTracker::GetRobotPose()
{
  return mRobotPose;
}

void HumanTracker::UpdateRobotPose(RobotPose &pose)
{
  if (mRobotSpeedStat) {
    double distance = (pose.mPosition - mRobotPose.mPosition).length();

    if (distance < 1.0) {
      STATISTIC::Add(mRobotSpeedStat, distance);
    }
  }

  mRobotPose = pose;
}

void HumanTracker::LogProposal(shared_ptr<Particles> &particles)
{
  if (mRCGLogger) {
    int pid = 0;
    foreach_(Particle::Ptr &p, *particles) {
      foreach_(HumanState::Ptr &hs, p->mHumans) {
        if (mDebugLevel > 8) {
          string str = StringPrintf("pid=%d", pid);
          mRCGLogger[LOG_ALL]->AddPoint(
              hs->Position().x, hs->Position().y,
              Qt::lightGray, str.c_str());
        }
        else {
          mRCGLogger[LOG_ALL]->AddPoint(
              hs->Position().x, hs->Position().y,
              Qt::lightGray);
        }

        vector2d vel = hs->Position() + 0.1 * hs->Velocity();
        mRCGLogger[LOG_ALL]->AddLine(
            hs->Position().x, hs->Position().y, vel.x, vel.y,
            Qt::lightGray);
      }
      ++pid;
    }
  }
}

void HumanTracker::LogParticles(
    shared_ptr<Particles> &particles, const char *tag)
{
  if (mDebugLevel > 1) {
    mTextLogger << "==============================================" << endl;
    mTextLogger << "Particles " << tag << ":" << endl;

    vector<pair<double, pair<int, Particle::Ptr> > > sorted;
    for (uint i = 0; i < particles->size(); ++i) {
      sorted.push_back(
          make_pair(
              (*particles)[i]->Likelihood(), make_pair(i, (*particles)[i])));
    }
    sort(sorted.begin(),
         sorted.end(),
         greater<pair<double, pair<int, Particle::Ptr> > >());

    for (uint i = 0; i < sorted.size(); ++i) {
      mTextLogger
          << "[pid=" << sorted[i].second.first
          << " #=" << i
          << " p=" << sorted[i].second.second->Likelihood()
          << " S=" << sorted[i].second.second->mHumans.size()
          << ": ";

      foreach_(HumanState::Ptr &h, sorted[i].second.second->mHumans) {
        mTextLogger << *h << " ";
      }

      mTextLogger << "]" << endl;
    }
    mTextLogger << endl;
  }
}

void HumanTracker::LogAssignments(int pid, Particle::Ptr &particle)
{
  if (mDebugLevel > 1) {
    mTextLogger << "Top Assignments pid=" << pid
        << " S=" << particle->mHumans.size()
        << " O=" << CurrentObservation().mDetections.size()
        << " #=" << particle->mAssignments->size()
        << " p=" << particle->mAssignments->total_prob
        << ":" << endl;

    int aid = 0;
    foreach_(Particle::Assignment::Ptr &assignment, *(particle->mAssignments)) {
      mTextLogger << "#" << aid
          << " p=" << assignment->prob()
          << " p1=" << assignment->prob1
          << " p2=" << assignment->prob2 << ": ";

      for (Particle::Assignment::iterator it = assignment->begin();
          it != assignment->end(); ++it) {
        mTextLogger << it->first << "->" << it->second << " ";
      }

      aid += 1;

      mTextLogger << endl;
    }

    mTextLogger << endl;
  }
}

void HumanTracker::LogAssignmentsRCG()
{
  if (mRCGLogger) {
    int pid = 0;
    foreach_(Particle::Ptr &p, *mParticles) {
      HashMap<HumanState*, int> h2o;
      Particle::Assignment::Ptr assignment = p->mAssignments->best();

      if (assignment) {
        for (Particle::Assignment::iterator it = assignment->begin();
            it != assignment->end(); ++it) {
          HumanState *hs = p->mHumans.get(it->second);
          h2o[hs] = it->first;
          vector2d &o = CurrentObservation().mDetections[it->first]->mPosition;
          vector2d &h = hs->Position();
          mRCGLogger[LOG_ALL]->LogLine(
              o.x, o.y, h.x, h.y, Qt::gray);
        }
      }

      foreach_(HumanState::Ptr &hs, p->mHumans) {
        if (!h2o.count(hs.get())) {
          vector2d &h = hs->Position();
          mRCGLogger[LOG_ALL]->AddPoint(h.x, h.y, Qt::gray);
        }
      }

      ++pid;
    }
  }
}

void HumanTracker::LogRCG(double duration)
{
  if (!mRCGLogger) {
    return;
  }

  const double text_margin =
      mRobotPose.mPosition.x + Params::ins().text_margin;
  const double text_position = mRobotPose.mPosition.y;

  for (int i = 0; i <= LOG_ALL; ++i) {
    mRCGLogger[i]->Focus(
        mRobotPose.mPosition.x, mRobotPose.mPosition.y);
  }

  for (int i = 0; i <= HumanTracker::LOG_ALL; ++i) {
    mRCGLogger[i]->AddPoint(
        mRobotPose.mPosition.x, mRobotPose.mPosition.y,
        Qt::black, "Robot");

    for (double d = 0.0; d <= 0.05; d += 0.01) {
    mRCGLogger[i]->LogCircle(
        mRobotPose.mPosition.x, mRobotPose.mPosition.y,
          0.25 - d, Qt::black);
    }

    if (Params::ins().view_width < 360.0) {
      vector2d eov1 = mRobotPose.mPosition
          + vector2d(Params::ins().view_length, 0.0).rotate(
              mRobotPose.mAngle - RAD(Params::ins().view_width / 2));
      vector2d eov2 = mRobotPose.mPosition
          + vector2d(Params::ins().view_length, 0.0).rotate(
              mRobotPose.mAngle + RAD(Params::ins().view_width / 2));

      mRCGLogger[i]->LogLine(
          mRobotPose.mPosition.x, mRobotPose.mPosition.y,
          eov1.x, eov1.y, Qt::black);
      mRCGLogger[i]->LogLine(
          mRobotPose.mPosition.x, mRobotPose.mPosition.y,
          eov2.x, eov2.y, Qt::black);
    }

//    if (mDebugLevel > 2) {
//      for (int i = 0; i < int(mTrajectory.size()) - 1; ++i) {
//        RobotPose *pose = &mTrajectory[i];
//        RobotPose *next_pose = &mTrajectory[i+1];
//
//        vector2d from = pose->mPosition;
//        vector2d to = next_pose->mPosition;
//        mRCGLogger[LOG_ALL]->LogLine(
//            from.x, from.y, to.x, to.y, Qt::black);
//      }
//    }
  }

//  if (mDebugLevel > 1) {
//    int pid = 0;
//    foreach_(Particle::Ptr &p, *mParticles) {
//      if (!p->mHumans.empty()) {
//        if (mDebugLevel > 7) {
//          stringstream ss;
//          ss << "pid=" << pid;
//          p->Log(mRCGLogger[LOG_ALL], true, ss.str().c_str());
//        }
//        else {
//          p->Log(mRCGLogger[LOG_ALL]);
//        }
//      }
//      ++pid;
//    }
//  }

  stringstream ss;
  ss << "ExpectedIdentifiedHumanCount: " << ExpectedIdentityCount();
  mRCGLogger[LOG_ALL]->AddText(
      text_margin, text_position + 0.1, Qt::black, ss.str().c_str());

  ss.str("");
  ss << "AverageHumanCount: " << AverageHumanCount();
  mRCGLogger[LOG_ALL]->AddText(
      text_margin, text_position + 0.25, Qt::black, ss.str().c_str());

  ss.str("");
  ss << "UpdateTimeInterval: " << duration << "s";
  mRCGLogger[LOG_ALL]->AddText(
      text_margin, text_position + 0.4, Qt::black, ss.str().c_str());

  ss.str("");
  ss << "ObservationSize: " << CurrentObservation().mDetections.size();
  mRCGLogger[LOG_OBS]->AddText(
      text_margin, text_position - 0.1, Qt::black, ss.str().c_str());
  mRCGLogger[LOG_STATE_OBS]->AddText(
      text_margin, text_position - 0.1, Qt::black, ss.str().c_str());
  mRCGLogger[LOG_ALL]->AddText(
      text_margin, text_position - 0.1, Qt::black, ss.str().c_str());

  ss.str("");
  ss << "Step: " << mCurrentStep;
  for (int i = 0; i <= LOG_ALL; ++i) {
    mRCGLogger[i]->AddText(
        text_margin, text_position - 1, Qt::black, ss.str().c_str());
  }

  if (mDebugLevel > 5) {
    LogAssignmentsRCG();
  }

  double log_position = text_position + 1.5;
  foreach_(IdentifiedHuman::Ptr &human, mIdentifiedHumans)  {
    LogIdentifiedHuman(human, log_position);
    log_position += 0.2;
  }
  {
    int loggers[] = {LOG_ALL, LOG_OBS, LOG_STATE_OBS};
    double radius[] = {/*0.01, 0.02, */0.25};

//    double delta = 0.35 * sqrt(2.0) / 2.0;
    int oid = 0;

    QColor color(Qt::black);
//    color.setAlphaF(0.5);

    foreach_(Detection::Ptr &o, CurrentObservation().mDetections) {
      foreach_(int logger, loggers) {
//        mRCGLogger[logger]->AddPoint(o->x(), o->y(), Qt::darkGray);

        foreach_(double r, radius) {
//          mRCGLogger[l]->AddCircle(o->x(), o->y(), r, color);

          mRCGLogger[logger]->LogRectangular(
              o->x() - r, o->x() + r, o->y() - r, o->y() + r, color);

//          vector2d topleft(o->x() - r, o->y() - r);
//          vector2d dir = vector2d(1.0, 1.0) / sqrt(2.0);
//          double len = 2.0 * r * sqrt(2.0);
//          double step = len / rint(10.0 * o->mConfidence);
//
//          for (double d = 0.0; d < len; d += step) {
//            vector2d c = topleft + d * dir;
//            double h = (d <= 0.5 * len)? d: len - d;
//            vector2d a = c + vector2d(1.0, -1.0) * h / sqrt(2.0);
//            vector2d b = c + vector2d(-1.0, 1.0) * h / sqrt(2.0);
//
//            mRCGLogger[logger]->AddLine(a.x, a.y, b.x, b.y, color);
//          }
        }

//        stringstream ss;
//        ss << "C=" << o->mConfidence;
//        ss << " O:" << oid;
//
//        mRCGLogger[l]->AddPoint(
//            o->x() + delta, o->y() + delta,
//            Qt::darkGray, ss.str().c_str());
      }

      ++oid;
    }
  }

//  for (double d = 0.0; d <= 0.05; d += 0.01) {
//    mRCGLogger[LOG_ALL]->LogRectangular(
//        -10.0 + d, -4.0 - d, 41.0 + d, 45.0 - d, Qt::black);
//  }

  for (int i = 0; i <= LOG_ALL; ++i) {
    mRCGLogger[i]->Flush();
  }
}

void HumanTracker::LogIdentifiedHuman(IdentifiedHuman::Ptr &human)
{
  if (mDebugLevel > 0) {
    mTextLogger << "Identified Human " << human->mID
        << " [C=" << human->Confidence() << "] :" << endl;

    mTextLogger << "IntentionDistribution "
        << human->mID << ": " << human->mIntentionDistri << endl;

    mTextLogger << "StatePool:" << endl;
    foreach_(HumanState *hs, human->mStatePool) {
      mTextLogger << *hs << endl;
    }
  }
}

void HumanTracker::LogIdentifiedHuman(
    IdentifiedHuman::Ptr &human, double text_position)
{
  if (!human->ExpectedState()) return;
  if (human->Confidence() < 0.1) return;
  double text_margin = mRobotPose.mPosition.x +
      Params::ins().text_margin;

  stringstream ss;
  ss << "Identified Human " << human->mID
      << " [Conf=" << human->Confidence() << "] [Speed="
      << human->ExpectedState()->Velocity().length() << "] :";
  mRCGLogger[LOG_ALL]->AddText(
      text_margin, text_position, Qt::black, ss.str().c_str());

//  {
//    double left = 4.3;
//
//    for (uint i = 0; i < IntentionFactory::ins().Size(); ++i) {
//      HumanIntention *intention = IntentionFactory::ins().GetIntention(i);
//      ss.str("");
//      ss << " [" << intention->GetName()
//          << ", " << human->mIntentionDistri[i] << "] ";
//      mRCGLogger[LOG_ALL]->AddText(
//          text_margin + left, text_position,
//          Qt::black,
//          ss.str().c_str());
//      left += 3.1;
//    }
//  }

//  mRCGLogger[LOG_ALL]->AddCircle(
//      human->ExpectedState()->Position().x,
//      human->ExpectedState()->Position().y, 0.02,
//      human->DisplayingColor());
//  mRCGLogger[LOG_ALL]->AddCircle(
//      human->ExpectedState()->Position().x,
//      human->ExpectedState()->Position().y, 0.03,
//      human->DisplayingColor());
//  mRCGLogger[LOG_ALL]->AddCircle(
//      human->ExpectedState()->Position().x,
//      human->ExpectedState()->Position().y, 0.3,
//      human->DisplayingColor());
//  vector2d vel = human->ExpectedState()->Velocity();
//  if (vel.length() > 0.0) {
//    vel *= 0.3 / vel.length();
//  }
//  vel += human->ExpectedState()->Position();
//  mRCGLogger[LOG_ALL]->AddLine(
//      human->ExpectedState()->Position().x,
//      human->ExpectedState()->Position().y,
//      vel.x, vel.y, human->DisplayingColor());

//  ss.str("");
//  ss << "Identified " << human->mID
//      << " C=" << human->Confidence();
//  double delta = 0.3 * sqrt(2.0) / 2.0;
//  mRCGLogger[LOG_ALL]->AddPoint(human->ExpectedState()->Position().x + delta,
//                                human->ExpectedState()->Position().y - delta,
//                                human->DisplayingColor(), ss.str().c_str());

  if (mDebugLevel > 5) {
    foreach_(HumanState *hs, human->mStatePool) {
      vector2d h = hs->Position();
      vector2d e = human->ExpectedState()->Position();
      mRCGLogger[LOG_ALL]->LogLine(h.x, h.y, e.x, e.y, Qt::lightGray);
    }
  }

  if (mDebugLevel > 1) {
    foreach_(HumanState *hs, human->mStatePool) {
      hs->Log(mRCGLogger[LOG_ALL]);
    }
  }

  if (mDebugLevel > 2) {
    for (int i = 0; i < int(human->mTrajectory.size()) - 1; ++i) {
      HumanState::Ptr &state = human->mTrajectory.at(i).state;
      HumanState::Ptr &next_state = human->mTrajectory.at(i+1).state;

      vector2d &from = state->Position();
      vector2d &to = next_state->Position();
      mRCGLogger[LOG_ALL]->LogLine(
          from.x, from.y, to.x, to.y, human->DisplayingColor());
    }
  }

//  if (mDebugLevel > 1) {
//    if (human->mDetection) {
//      vector2d &e = human->ExpectedState()->Position();
//      vector2d &o = human->mDetection->mPosition;
//      mRCGLogger[LOG_ALL]->LogLine(e.x, e.y, o.x, o.y, Qt::darkGray);
//    }
//  }

  for (uint i = 0; i < IntentionFactory::ins().Size(); ++i) {
    foreach_(const HumanState::Ptr &hs, human->mParticlesGivenIntention[i]) {
      hs->Log(mRCGLogger[LOG_STATE]);
      hs->Log(mRCGLogger[LOG_STATE_OBS]);
    }
  }

  {
     mRCGLogger[LOG_ALL]->AddPoint(
         human->ExpectedState()->Position().x,
         human->ExpectedState()->Position().y, Qt::black);

     vector2d delta = 0.1 * vector2d(1, 1) / sqrt(2.0);

     for (int i = 0; i < 4; ++i) {
       vector2d end = human->ExpectedState()->Position() + delta;
       vector2d dir = delta.rotate(RAD(90.0));
       int n = rint(15 * human->Confidence());
       double d = 0.01;

       for (double step = -d * n; step <= d * n; step += d) {
         vector2d adj = dir * step;
         mRCGLogger[LOG_ALL]->AddLine(
             human->ExpectedState()->Position().x + adj.x,
             human->ExpectedState()->Position().y + adj.y,
             end.x + adj.x, end.y + adj.y, Qt::black);
       }

       delta = delta.rotate(RAD(90.0));
    }
  }
}

/**
 * update intention distribution
 * conditioned on state hs
 * after intention transition
 * for identified human ih
 */
void HumanTracker::UpdateIntentionDistribution(
    IdentifiedHuman::Ptr &ih, HumanState &hs)
{
//  hs.mIntentionDistri.Assertion();
//
//  for (uint i = 0; i < IntentionFactory::ins().Size(); ++i) {
//    double density = KernelDensityEstimation(
//        hs.Position(), &(ih->mKernelsGivenIntention[i]));
//
//    hs.mIntentionDistri[i] *= density;
//  }
//
//  hs.mIntentionDistri /= hs.mIntentionDistri.sum();
//  hs.mIntentionDistri.Assertion();
}

void HumanTracker::BuildIntentionDistribution(IdentifiedHuman::Ptr &human)
{
  if (human->mIntentionDistri.isZero()) {
    human->mIntentionDistri = IntentionDistribution::RandomUniform();
  }

  if (human->Age() == 0) {
    foreach_(HumanState *hs, human->mStatePool) {
      hs->mIntentionDistri = IntentionDistribution::Random();
    }
  }

  if (Params::ins().mixed_filters) {
    foreach_(HumanState *hs, human->mStatePool) {
      UpdateIntentionDistribution(human, *hs);
    }
  }

  human->mIntentionDistri.setZero();
  foreach_(HumanState *hs, human->mStatePool) {
    human->mIntentionDistri += hs->mIntentionDistri;
  }

  human->mIntentionDistri /= human->mStatePool.size();
  human->mIntentionDistri.Assertion();
}

HumanIntention *HumanTracker::GetDominantIntention(
    HumanIntention *intention,
    IntentionDistribution &intentions)
{
  vector<pair<double, HumanIntention*> > sorted;

  for (uint i = 0; i < IntentionFactory::ins().Size(); ++i) {
    HumanIntention *intention = IntentionFactory::ins().GetIntention(i);
    double prob = intentions[i];
    sorted.push_back( make_pair(prob, intention) );
  }

  sort(sorted.begin(), sorted.end(),
       greater<pair<double, HumanIntention*> >());

  if (sorted.empty()) {
    return 0;
  }
  else if (sorted.size() == 1) {
    return sorted[0].second;
  }
  else {
    double threshold = (intention == sorted[0].second)?
        0.5: mDominantIntentionRate;
    return (sorted[0].first > threshold)? sorted[0].second: 0;
  }
}

void HumanTracker::ClearGhosts(
    vector<IdentifiedHuman::Ptr> &identified_humans,
    HashMap<int, HashMap<int, double> > &hi_prob,
    std::vector<Particle::Ptr> &particles)
{
  vector<IdentifiedHuman::Ptr>::iterator it = identified_humans.begin();

  while (it != identified_humans.end()) {
    double sum = 0.0;
    foreach_(Particle::Ptr &p, particles) {
      foreach_(HumanState::Ptr &hs, p->mHumans) {
        sum += hi_prob[hs->mID][(*it)->mID];
        if (sum >= mIdentificationMinProb) {
          break;
        }
      }
      if (sum >= mIdentificationMinProb) {
        break;
      }
    }

    if (sum < mIdentificationMinProb) {
      it = identified_humans.erase(it);
      continue;
    }
    ++it;
  }
}

void HumanTracker::ParticleFilteringWorker::StartRoutine()
{
  mObservation = CachedObservation::Ptr(
      new CachedObservation(mTracker->CurrentObservation()));

  for (uint i = 0; i < Params::ins().num_particles; ++i) {
    if (i % mThreads == mID) {
      (*mTracker->mRefinedParticles)[i] =
          (*mTracker->mParticles)[i]->Update(*mObservation, mDuration);
    }
  }
}

void HumanTracker::ParticleFiltering(double duration)
{
  if (Params::ins().mixed_filters) {
    foreach_(IdentifiedHuman::Ptr &ih, mIdentifiedHumans) {
      ih->BuildKernerlsGivenIntention(duration);
    }
  }

  CurrentObservation().Reset();

  if (Params::ins().threads) {
    vector<ParticleFilteringWorker::Ptr> workers;

    for (uint i = 0; i < Params::ins().threads; ++i) {
      workers.push_back(make_shared<ParticleFilteringWorker>(
          this, duration, i, Params::ins().threads));
    }

    foreach_(ParticleFilteringWorker::Ptr &p, workers) {
      p->Start();
    }

    foreach_(ParticleFilteringWorker::Ptr &p, workers) {
      p->Join();
    }
  }
  else {
    ParticleFilteringWorker worker(this, duration);
    worker.StartRoutine();
  }

  LogProposal(mRefinedParticles);
  LogParticles(mRefinedParticles, "proposed");

  bool refinement = false;
  for (uint i = 0; i < Params::ins().num_particles; ++i) {
    if ((*mRefinedParticles)[i] != (*mParticles)[i]) {
      refinement = true;
      break;
    }
  }

  if (refinement) {
    ComputePosteriorPredictive(mParticles);
    ComputePosteriorPredictive(mRefinedParticles);

    vector<int> selected;
    {
      uint kernels = Min(
          mNegativeBinomial[mParticles.get()].first,
          mNegativeBinomial[mRefinedParticles.get()].first) - mNegativeBinomialAlpha;

      if (kernels <= Params::ins().position_kernel_size) {
        for (uint i = 0; i < Params::ins().num_particles; ++i) {
          selected.push_back(i);
        }
      }
      else {
        uint kernels0 = 0;
        uint kernels1 = 0;

        while (min(kernels0, kernels1) <= Params::ins().position_kernel_size) {
          int i = SimpleRNG::ins().GetRand() % Params::ins().num_particles;
          selected.push_back(i);

          kernels0 += (*mParticles)[i]->mHumans.size();
          kernels1 += (*mRefinedParticles)[i]->mHumans.size();
        }
      }
    }

    BuildPositionKernels(mParticles, selected);
    BuildPositionKernels(mRefinedParticles, selected);

    if (Params::ins().threads) {
      vector<KernelDensityWorker::Ptr> workers;

      for (uint i = 0; i < Params::ins().threads; ++i) {
        workers.push_back(make_shared<KernelDensityWorker>(
            this, i, Params::ins().threads));
      }

      foreach_(KernelDensityWorker::Ptr &p, workers) {
        p->Start();
      }

      foreach_(KernelDensityWorker::Ptr &p, workers) {
        p->Join();
      }
    }
    else {
      KernelDensityWorker worker(this);
      worker.StartRoutine();
    }

    LogParticles(mRefinedParticles, "refined");
  }

  Normalize(mRefinedParticles);
  LogParticles(mRefinedParticles, "normalized");

  LowVarianceResample(mRefinedParticles, mParticles);
  LogParticles(mParticles, "resampled");

  AddNoise(mParticles);
  LogParticles(mParticles, "noise added");
}

void HumanTracker::KernelDensityWorker::StartRoutine()
{
  int pid = 0;
  foreach_(Particle::Ptr &p, *(mTracker->mRefinedParticles)) {
    if (pid++ % mThreads == mID) {
      double motion_weight = mTracker->ComputeMotionWeight(*p, mCache);
      double proposal_weight = mTracker->ComputeProposalWeight(*p, mCache);
      double ratio = motion_weight / proposal_weight;

      p->Likelihood() *= ratio;
    }
  }
}

void HumanTracker::IntentionRecgonition(double duration)
{
  foreach_(IdentifiedHuman::Ptr &human, mIdentifiedHumans) {
    assert(human->ExpectedState());

    if (mRoot && Params::ins().hierarchical_filters) {
      if (!human->mIntentionTracker) {
        human->CreateIntentionTracker(mTask, mDebugLevel);
      }

      Observation::Ptr obs = make_shared<Observation>();

      if (human->mDetection) {
        obs->mDetections.push_back(human->mDetection);
      }
      else {
        obs->mDetections.push_back(
            make_shared<Detection>(
                human->ExpectedState()->Position(),
                human->ExpectedState()->Orientation(),
                1.0, -1.0, -1.0, -1.0, -1.0));
      }

      human->mIntentionTracker->UpdateRobotPose(mRobotPose);
      human->mIntentionTracker->Update(obs, duration);

      if (human->mIntentionTracker->IdentifiedHumans().size() &&
          human->mIntentionTracker->IdentifiedHumans().front()->ExpectedState() ) {
        IdentifiedHuman::Ptr shadow = human->mIntentionTracker->IdentifiedHumans().front();

        human->mIntentionDistri = shadow->mIntentionDistri;
        human->mIntentionDistri.Assertion();

        human->mIntention = shadow->mIntention;

        if (shadow->ExpectedState()) {
          human->ExpectedState()->CopyFrom(*shadow->ExpectedState());

          if (!shadow->mStatePool.empty()) {
            vector<const HumanState*> states;

            foreach_(HumanState *hs, shadow->mStatePool) {
              assert(hs->mIdentity == shadow.get());
              states.push_back(hs);
            }

            foreach_(HumanState *hs, human->mStatePool) {
              assert(hs->mIdentity == human.get());
              hs->CopyFrom(*states.at(SimpleRNG::ins().GetRand() % states.size()));
            }
          }
        }
      }
    }
    else {
      BuildIntentionDistribution(human);

      human->mIntention =
          GetDominantIntention(
              human->mIntention, human->mIntentionDistri);
    }

    LogIdentifiedHuman(human);
  }
}

void HumanTracker::UpdateIdentifiedHumans()
{
  {
    vector<IdentifiedHuman::Ptr>::iterator it = mIdentifiedHumans.begin();
    while (it != mIdentifiedHumans.end()) {
      if ((*it)->mStatePool.empty()) {
        it = mIdentifiedHumans.erase(it);
        continue;
      }

      ++it;
    }
  }

  if (mIdentifiedHumans.empty()) {
    return;
  }

  foreach_(IdentifiedHuman::Ptr &human, mIdentifiedHumans) {
    {
      HumanState::Ptr state = make_shared<HumanState>();

      state->SetDetection(human->mDetection);
      state->SetIntention(human->mIntention);
      human->mTrajectory.push_back(
          IdentifiedHuman::Track(
              state, human->Confidence(), mRobotPose));

      vector<IdentifiedHuman::Track>::iterator it = human->mTrajectory.begin();
      while (human->mTrajectory.size() > Params::ins().trajectory_size) {
        it = human->mTrajectory.erase(it);
      }
    }

    vector2d position = vector2d(0.0, 0.0);
    vector2d velocity = vector2d(0.0, 0.0);
    double orientation = 0.0;

    if (!human->mStatePool.empty()) {
      double orients[5] = {0, 0, 0, 0, 0};
      foreach_(HumanState *hs, human->mStatePool) {
        position += hs->Position();
        velocity += hs->Velocity();
        if (hs->Orientation() != DBL_MAX) {
          double orient_local = hs->Orientation() - mRobotPose.mAngle;
          orient_local = _angle::GetNormalizeAngleRad(orient_local, 0);  // restrict to 0-2pi
          int bin = (int) round(orient_local / (M_PI / 2.0));
          if (bin >= 0 && bin <= 4) {
            orients[bin] += 1;
          }
          else {
            assert(0);
          }
        } else {
          orients[4] += 1;
        }
      }

      position /= human->mStatePool.size();
      velocity /= human->mStatePool.size();
      int n = 0;
      for(int i = 0; i < 4; i++) {
        if(orients[i] >= n) {
          orientation = i * (M_PI / 2.0) + mRobotPose.mAngle;  // in world frame
          n = orients[i];
        }
      }

      if(orients[4] > n) {
        orientation = DBL_MAX;
      }

      if (isinf(orientation) || isnan(orientation)) {
        assert(0);
      }
    }

    human->ExpectedState()->SetPosition(position);
    human->ExpectedState()->SetVelocity(velocity);
    human->ExpectedState()->SetOrientation(orientation);

    STATISTIC::Add(mIdentifiedHumanConfidence, human->Confidence());
    bool approaching = human->Approaching(Params::ins().approaching_samples);
    if (approaching && !human->mApproaching) {
      if (mRoot) {
        // speak("hello human");
      }
      human->mApproaching = approaching;
    }
    //else if (approaching && !human->mApproaching) {
    //  speak("goodbye human");
    //}
  }

  if (!mIdentifiedHumans.empty()) {
    STATISTIC::Add(mAverageHumanCountStat, AverageHumanCount());
    STATISTIC::Add(mIdentityCountStat, mIdentifiedHumans.size());
    STATISTIC::Add(mExpectedIdentityCountStat, ExpectedIdentityCount());
  }
}

void HumanTracker::BuildStatePools()
{
  foreach_(IdentifiedHuman::Ptr &human, mIdentifiedHumans) {
    human->mStatePool.clear();
  }

  foreach_(Particle::Ptr &p, *mParticles) {
    foreach_(HumanState::Ptr &hs, p->mHumans) {
      if (hs->mIdentity) {
        hs->mIdentity->mStatePool.insert(hs.get());
      }
    }
  }

//  if (mDebugLevel > 0) {
//    foreach_(IdentifiedHuman::Ptr &i, mIdentifiedHumans) {
//      foreach_(HumanState *h, i->mStatePool) {
//        assert(h->mIdentity == i.get());
//      }
//    }
//
//    foreach_(Particle::Ptr &particle, *mParticles) {
//      Particle::HumanList::iterator it = particle->mHumans.begin();
//
//      foreach_(HumanState::Ptr &h, particle->mHumans) {
//        if (h->mIdentity == 0) {
//          foreach_(IdentifiedHuman::Ptr &i, mIdentifiedHumans) {
//            assert(i->mStatePool.count(h.get()) == 0);
//          }
//        }
//      }
//    }
//  }
}

void HumanTracker::HumanIdentification(double duration)
{
  foreach_(IdentifiedHuman::Ptr &ih, mIdentifiedHumans) {
    ih->mDetection = Detection::Ptr();
  }

  if (!CurrentObservation().mDetections.empty()) {
    HumanIdentificationImp(duration);
  }

  BuildStatePools();
  UpdateIdentifiedHumans();
}

void HumanTracker::HumanIdentificationWorker::StartRoutine()
{
  for (uint p = 0; p < Params::ins().num_particles; ++p) {
    if (p % mThreads == mID) {
      Particle::Ptr &particle = (*mTracker->mParticles)[p];

      if (particle->mHumans.empty()) {
        continue;
      }

      vector<IdentifiedHuman::Ptr> identified_humans = mIdentifiedHumans;
      if (identified_humans.size() > particle->mHumans.size()) { //reduce cols
        vector<Particle::Ptr> particles;
        particles.push_back(particle);
        mTracker->ClearGhosts(identified_humans, mProbMat, particles);
      }

      while (identified_humans.size() < particle->mHumans.size()) { //increase cols
        identified_humans.push_back(IdentifiedHuman::Ptr());
      }

      HashMap<int, HashMap<int, double> > prob_mat;
      const uint humans = particle->mHumans.size();

      foreach_(HumanState::Ptr &hs, particle->mHumans) {
        for (uint i = 0; i < identified_humans.size(); ++i) {
          double prob = identified_humans[i]?
              mProbMat[hs->mID][identified_humans[i]->mID]: 0.0;
          prob_mat[hs->mID][i] = prob;
        }
      }

      Particle::Permutation::Ptr perm = Murty::ins().Solve(
          prob_mat, identified_humans.size(), *particle);

      if (perm->size() < humans) {
        assert(0);
        continue;
      }

      for (uint i = 0; i < humans; ++i) {
        HumanState *hs = particle->mHumans.get(i);
        uint j = perm->at(i);

        if (identified_humans[j] &&
            prob_mat[hs->mID][j] > mIdentificationMinProb) {
          mConverged = mConverged && hs->mIdentity == identified_humans[j].get();
          hs->mIdentity = identified_humans[j].get();
        }
        else {
          mConverged = mConverged && hs->mIdentity == 0;
          hs->mIdentity = 0;
        }
      }
    }
  }
}

void HumanTracker::HumanIdentificationImp(double duration)
{
  HashMap<int, HumanStatePool> o2h;
  HashMap<int, int> h2o;

  foreach_(Particle::Ptr &p, *mParticles) {
    Particle::Assignment::Ptr assignment = p->mAssignments->propose();

    if (assignment) {
      for (Particle::Assignment::iterator it = assignment->begin();
          it != assignment->end(); ++it) {
        int o = it->first;
        HumanState *hs = p->mHumans.get(it->second);

        o2h[o].insert(hs);
        h2o[hs->mID] = o;
        hs->SetDetection(CurrentObservation().mDetections[o]);
      }
    }
  }

  for (uint o = 0; o < CurrentObservation().mDetections.size(); ++o) {
    IdentifiedHuman::Ptr ih = make_shared<IdentifiedHuman>();

    mIdentifiedHumans.push_back(ih);
    ih->mDetection = CurrentObservation().mDetections[o];
  }

  if (mIdentifiedHumans.empty()) {
    return;
  }

  HashMap<int, HashMap<int, double> > oi_prob;
  uint em_steps = 0, converged;

  do {
    converged = true;

    BuildStatePools();
    oi_prob.clear();

    HashMap<int, double> i_prob;
    HashMap<int, double> o_prob;

    foreach_(IdentifiedHuman::Ptr &ih, mIdentifiedHumans) {
      if (!ih->mDetection) {
        for (uint o = 0; o < CurrentObservation().mDetections.size(); ++o) {
          double count = 0.0;

          foreach_(HumanState *hs, o2h[o]) {
            if (hs->mIdentity == ih.get()) {
              count += 1.0;
            }
          }

          double prob = count / Params::ins().num_particles;

          if (prob > 0.0) {
            oi_prob[o][ih->mID] = prob;
            i_prob[ih->mID] += prob;
            o_prob[o] += prob;
          }
        }
      }
    }

    foreach_(IdentifiedHuman::Ptr &ih, mIdentifiedHumans) {
      if (ih->mDetection) {
        for (uint o = 0; o < CurrentObservation().mDetections.size(); ++o) {
          if (ih->mDetection == CurrentObservation().mDetections[o]) {
            double conf = double(o2h[o].size()) / Params::ins().num_particles;

            if (conf > 0.0) {
              double prob = conf - o_prob[o];
              assert(prob >= 0.0);

              prob = (prob >=
                  Params::ins().observation_proposal_prob * conf)? prob: 0.0;

              if (prob > 0.0) {
                oi_prob[o][ih->mID] = prob;
                i_prob[ih->mID] += prob;
                o_prob[o] += prob;
              }
            }
          }
        }
      }
    }

    if (mDebugLevel > 0) {
      mTextLogger << "EM round " << em_steps << ":" << endl;
      for (uint o = 0; o < CurrentObservation().mDetections.size(); ++o) {
        foreach_(IdentifiedHuman::Ptr &ih, mIdentifiedHumans) {
          if (oi_prob[o][ih->mID] > 0.0) {
            mTextLogger << "oi_prob[" << o << "][" << ih->mID << "] = "
                << oi_prob[o][ih->mID] << endl;
          }
        }
      }
      mTextLogger << endl;
    }

    HashMap<int, HashMap<int, double> > hi_prob;

    foreach_(Particle::Ptr &p, *mParticles) {
      foreach_(HumanState::Ptr &hs, p->mHumans) {
        int o = h2o.count(hs->mID)? h2o[hs->mID]: -1;

        foreach_(IdentifiedHuman::Ptr &ih, mIdentifiedHumans) {
          if (o != -1) {
            hi_prob[hs->mID][ih->mID] = oi_prob[o][ih->mID];
          }
          else if (hs->mIdentity == ih.get()) {
            double prob = ih->Confidence() - i_prob[ih->mID];
            assert(prob >= 0.0);
            hi_prob[hs->mID][ih->mID] = Max(prob, 0.0);
          }
          else {
            hi_prob[hs->mID][ih->mID] = 0.0;
          }
        }
      }
    }

    vector<IdentifiedHuman::Ptr> identified_humans_ = mIdentifiedHumans;
    ClearGhosts(identified_humans_, hi_prob, *mParticles);

    if (Params::ins().threads) {
      vector<HumanIdentificationWorker::Ptr> workers;

      for (uint i = 0; i < Params::ins().threads; ++i) {
        workers.push_back(HumanIdentificationWorker::Ptr(
            new HumanIdentificationWorker(
                this, identified_humans_, hi_prob, i, Params::ins().threads)));
      }

      foreach_(HumanIdentificationWorker::Ptr &p, workers) {
        p->Start();
      }

      foreach_(HumanIdentificationWorker::Ptr &p, workers) {
        p->Join();
        converged = converged && p->mConverged;
      }
    }
    else {
      HumanIdentificationWorker worker(this, identified_humans_, hi_prob);
      worker.StartRoutine();
      converged = worker.mConverged;
    }

  } while (++em_steps < Params::ins().max_em_steps && !converged);

  STATISTIC::Add(mEMConvergedSteps, em_steps);

  Particle::Permutation::Ptr perm = Murty::ins().Solve(
      oi_prob, CurrentObservation().mDetections.size(), mIdentifiedHumans);

  if (perm->size() >= CurrentObservation().mDetections.size()) {
    for (uint o = 0; o < CurrentObservation().mDetections.size(); ++o) {
      IdentifiedHuman *ih = mIdentifiedHumans[perm->at(o)].get();

      if (oi_prob[o][ih->mID] > mIdentificationMinProb) {
        ih->mDetection = CurrentObservation().mDetections[o];
      }
    }
  }
}

double HumanTracker::ComputeMotionWeight(
    Particle &particle, KernelDensityCache &cache)
{
  return OverallDensityEstimation(particle, mParticles, cache);
}

double HumanTracker::ComputeProposalWeight(
    Particle &particle, KernelDensityCache &cache)
{
  return OverallDensityEstimation(particle, mRefinedParticles, cache);
}

double HumanTracker::OverallDensityEstimation(
    Particle &query, shared_ptr<Particles> &support, KernelDensityCache &cache)
{
  boost::math::negative_binomial_distribution<double> dist(
      mNegativeBinomial[support.get()].first,
      mNegativeBinomial[support.get()].second);

  double prob = boost::math::pdf(dist, query.mHumans.size());
  foreach_(HumanState::Ptr &hs, query.mHumans) {
    prob *= KernelDensityEstimation(
        hs->Position(), &mPositionKernels[support.get()], cache);
  }
  prob *= Combination::ins().Factorial(query.mHumans.size());

  return prob;
}

double HumanTracker::KernelDensityEstimation(
    const vector2d &query, Kernels *support, KernelDensityCache &cache)
{
  pair<int, int> key = PositionToIndex(query);

  if (cache[support].count(key.first) &&
      cache[support][key.first].count(key.second)) {
    return cache[support][key.first][key.second];
  }

  double prob = 0.0;

  int computed_kernels = 0;

  if (!support->empty()) {
    foreach_(vector2d *s, *support) {
      double len = (query - *s).length();

      if (len < mKernelSize) {
        prob += Gaussian::ins().pdf(0.0, 1.0, len / mKernelBandwidth);
        computed_kernels += 1;
      }
    }

    prob /= support->size() * mKernelBandwidth;
  }

  STATISTIC::Add(mComputedKernelsStat, computed_kernels);
  STATISTIC::Add(mAllKernelsStat, support->size());
  prob += mBackgroundDensity;

  cache[support][key.first][key.second] = prob;

  return prob;
}

void HumanTracker::ComputePosteriorPredictive(shared_ptr<Particles> &support)
{
  double alpha = mNegativeBinomialAlpha;

  foreach_(Particle::Ptr &p, *support) {
    alpha += p->mHumans.size();
  }

  double beta = mNegativeBinomialBeta + support->size();

  mNegativeBinomial[support.get()] = make_pair(alpha, beta / (beta + 1.0));
}

void HumanTracker::BuildPositionKernels(
    shared_ptr<Particles> &support, std::vector<int> &selected)
{
  foreach_(int i, selected) {
    foreach_(HumanState::Ptr &hs, (*support)[i]->mHumans) {
      mPositionKernels[support.get()].push_back(&(hs->Position()));
    }
  }
}

double HumanTracker::AverageHumanCount()
{
  double ret = 0.0;

  foreach_(Particle::Ptr &p, *mParticles) {
    ret += p->mHumans.size();
  }

  ret /= Params::ins().num_particles;

  return ret;
}

double HumanTracker::ExpectedIdentityCount()
{
  double ret = 0.0;

  foreach_(IdentifiedHuman::Ptr &human, mIdentifiedHumans) {
    ret += human->Confidence();
  }

  return ret;
}

