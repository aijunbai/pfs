/*
 * elevator.cpp
 *
 *  Created on: Jan 10, 2014
 *      Author: baj
 */

#include <boost/math/distributions/negative_binomial.hpp>
#include <boost/math/distributions/normal.hpp>
#include <elevator_entering.h>
#include "tracker.h"
#include "params.h"
#include "tracker.h"

#include "line.h"

using namespace std;

namespace {
bool ret1 = Task::Register<ElevatorEnteringTask_SL>( "ElevatorEnteringTask_SL");
bool ret2 = Task::Register<ElevatorEnteringTask_SLH>("ElevatorEnteringTask_SLH");
}

const double ElevatorEnteringTaskBase::mElevatorLeft = 2.75;
const double ElevatorEnteringTaskBase::mElevatorRight = 5.0;
const double ElevatorEnteringTaskBase::mElevatorTop = -1.5;
const double ElevatorEnteringTaskBase::mElevatorBottom = 1.5;
const double ElevatorEnteringTaskBase::mEnvironmentTop = -4.0;
const double ElevatorEnteringTaskBase::mEnvironmentBottom = 4.0;
const double ElevatorEnteringTaskBase::mWallWidth = 0.15;

const double ElevatorEnteringTaskBase::ElevatorSimulator::mExpectedDoorOpenTime = 10.0;
const QColor ElevatorEnteringTaskBase::ElevatorSimulator::mWallColor = Qt::blue;

ElevatorEnteringTaskBase::ElevatorEnteringTaskBase():
    Task("ElevatorEnteringTask")
{

}

ElevatorEnteringTaskBase::ElevatorEnteringTaskBase(std::string name): Task(name)
{

}

ElevatorEnteringTaskBase::~ElevatorEnteringTaskBase()
{
}

ElevatorEnteringTaskBase::Stay::Stay(HumanTracker *tracker):
    HumanIntention("ElevatorEnteringTask::Stay", tracker)
{

}

ElevatorEnteringTaskBase::Stay::~Stay()
{
}

shared_ptr<HumanAction> ElevatorEnteringTaskBase::Stay::ChooseAction(
    const HumanState &hs)
{
  return make_shared<_motion::Stay>();
}

ElevatorEnteringTaskBase::Leave::Leave(HumanTracker *tracker):
    HumanIntention("ElevatorEnteringTask::Leave", tracker)
{

}

ElevatorEnteringTaskBase::Leave::~Leave()
{
}

shared_ptr<HumanAction> ElevatorEnteringTaskBase::Leave::ChooseAction(
    const HumanState &hs)
{
  double power = fabs(_motion::RandomPower());
  double angle = GetTracker()->GetRobotPose().mAngle + RAD(180.0);

  angle += _motion::NoiseAngle(45.0);

  return make_shared<_motion::Dash>(power, angle);
}

ElevatorEnteringTaskBase::Hesitate::Hesitate(HumanTracker *tracker):
    HumanIntention("ElevatorEnteringTask::Hesitate", tracker)
{

}

ElevatorEnteringTaskBase::Hesitate::~Hesitate()
{
}

shared_ptr<HumanAction>
ElevatorEnteringTaskBase::Hesitate::ChooseAction(
    const HumanState &hs)
{
  return IntentionFactory::ins().GetIntention(
      "ElevatorEnteringTask::Stay")->ChooseAction(hs);
}

HumanIntention *ElevatorEnteringTaskBase::Hesitate::ChangeIntention(
    const HumanState &hs, double duration)
{
  double prob = 1.0 - pow(Params::ins().change_intention_rate, duration);

  if (SimpleRNG::ins().GetUniform() < prob) {
    return IntentionFactory::ins().GetIntention(
        "ElevatorEnteringTask::Leave");
  }

  return this;
}

HashMap<HumanIntention*, double> ElevatorEnteringTaskBase::Hesitate::GetTransitions(
    const HumanState &hs, double duration)
{
  HashMap<HumanIntention*, double> trans;

  double prob = 1.0 - pow(Params::ins().change_intention_rate, duration);

  trans[this] = 1.0 - prob;
  trans[IntentionFactory::ins().GetIntention(
      "ElevatorEnteringTask::Leave")] = prob;

  return trans;
}

vector2d ElevatorEnteringTaskBase::NewHumanPosition()
{
  return _motion::RandomPosition(
      LeftInnerBoundary() + mWallWidth,
      mElevatorRight, mElevatorTop, mElevatorBottom);
}

shared_ptr<Simulator> ElevatorEnteringTaskBase::CreateSimulator()
{
  return make_shared<ElevatorSimulator>(this, GetTracker());
}

ElevatorEnteringTaskBase::ElevatorSimulator::ElevatorSimulator(
    Task *task, HumanTracker *tracker):
        Simulator(task, tracker), mHumanCount(0)
{
  Params::ins().false_rate = 0.5;
  Params::ins().missing_rate = 0.5;
  Params::ins().death_rate = 0.02;

  Params::ins().max_humans = 5;

  mHumanCount = 0;
  mDoorPosition = 0.0;
  mDoorDirection = 0.0;
  mDoorOpenTime =
      mTracker->mCurrentStep +
      SimpleRNG::ins().GetExponential(1.0 / mExpectedDoorOpenTime);
  mDoorCloseTime = mDoorOpenTime + 100;

  mWalls.clear();
  mWalls.push_back(line2d(vector2d(mElevatorLeft, mElevatorTop),
                          vector2d(mElevatorRight, mElevatorTop)));
  mWalls.push_back(line2d(vector2d(mElevatorRight, mElevatorTop),
                          vector2d(mElevatorRight, mElevatorBottom)));
  mWalls.push_back(line2d(vector2d(mElevatorLeft, mElevatorBottom),
                          vector2d(mElevatorRight, mElevatorBottom)));
  mWalls.push_back(line2d(vector2d(LeftInnerBoundary(), mElevatorTop),
                          vector2d(LeftInnerBoundary(), 0)));

  mCenter = vector2d((mElevatorLeft + mElevatorRight) / 2,
                     (mElevatorTop + mElevatorBottom) / 2);
}

ElevatorEnteringTaskBase::ElevatorSimulator::~ElevatorSimulator()
{
}

void ElevatorEnteringTaskBase::ElevatorSimulator::Initialize(double expected_humans)
{
  mHumanCount = 0;

  if (expected_humans > 0.0) {
    do {
      mHumanCount = SimpleRNG::ins().GetPoisson(expected_humans);
    } while (mHumanCount == 0);
  }

  mDoorPosition = 0.0;
  mDoorDirection = 0.0;
  mDoorOpenTime =
      mTracker->mCurrentStep +
      SimpleRNG::ins().GetExponential(1.0 / mExpectedDoorOpenTime);
  mDoorCloseTime = mDoorOpenTime + 100;

  mHumans.clear();

  for (uint i = 0; i < mHumanCount; ++i) {
    HumanIntention *intention =
        IntentionFactory::ins().GetRandomIntention();
    mHumans.push_back(make_shared<Human>(
        mTask->NewHumanPosition(),
        _motion::RandomPosition(
            -7.0,
            mElevatorLeft,
            mEnvironmentTop,
            mEnvironmentBottom),
            intention));
  }
}

void ElevatorEnteringTaskBase::ElevatorSimulator::CheckConsistence(Observation &obs)
{
  {
    list<Human::Ptr>::iterator it = mHumans.begin();
    while (it != mHumans.end()) {
      if ((*it)->Position().x < 0.0) {
        it = mHumans.erase(it);
      }
      else {
        ++it;
      }
    }
  }

  foreach_(Human::Ptr &human, mHumans) {
    if (!human->mTrajectory.empty()) {
      vector2d &last_pos = human->mTrajectory.back();

      if (EdgeCrossesDoor(human->Position(), last_pos)
          || EdgeCrossesWalls(human->Position(), last_pos)) {
        human->SetPosition(last_pos);
      }
    }
    else {
      while (EdgeCrossesDoor(human->Position(), mCenter)
          || EdgeCrossesWalls(human->Position(), mCenter)) {
        human->SetPosition(human->Position() - (human->Position() - mCenter) / 10.0);
      }
    }
  }

  {
    std::vector<Detection::Ptr>::iterator it = obs.mDetections.begin();
    while (it != obs.mDetections.end()) {
      if (!InFieldOfView((*it)->mPosition)) {
        it = obs.mDetections.erase(it);
        continue;
      }
      ++it;
    }
  }
}

bool ElevatorEnteringTaskBase::ElevatorSimulator::End()
{
  return mTracker->mCurrentStep >= mDoorCloseTime && mDoorPosition >= 0;
}

void ElevatorEnteringTaskBase::ElevatorSimulator::Simulate(
    double dt, double birth_rate, double death_rate)
{
  if (mTracker->mCurrentStep >= mDoorCloseTime) {
    bool crossing = false;

    foreach_(Human::Ptr &human, mHumans) {
      double x = human->Position().x;

      if (LeftOuterBoundary() <= x && x <= LeftInnerBoundary()) {
        crossing = true;
        break;
      }
    }

    if (crossing) {
      mDoorDirection = -1.0;
    }
    else {
      mDoorDirection = 1.0;
    }
  }
  else if (mTracker->mCurrentStep >= mDoorOpenTime) {
    mDoorDirection = -1.0;
  }

  mDoorPosition += mDoorDirection * 0.5 * dt;

  if (mDoorPosition > 0) {
    mDoorPosition = 0;
  }

  if (mDoorPosition < mElevatorTop) {
    mDoorPosition = mElevatorTop;
  }

  foreach_(Human::Ptr &human, mHumans) {
    HumanIntention *intention = human->Intention();

    if (intention->GetName() == "ElevatorEnteringTask::Leave") {
      if (human->Position().x < LeftInnerBoundary()) {
        human->Predict(dt);
      }
      else {
        vector2d target =
            _motion::RandomPosition(
                LeftInnerBoundary(), LeftInnerBoundary(),
                0.0, mElevatorBottom);
        double angle = (target - human->Position()).angle();
        double power = SimpleRNG::ins().GetNormal(0.5, 0.05);

        _motion::Dash(power, angle).Apply(*human, dt);
      }
    }
    else if (intention->GetName() == "ElevatorEnteringTask::Hesitate") {
      if (mDoorPosition <= mElevatorTop) {
        human->Predict(dt);
      }
    }
    else {
      human->Predict(dt);
    }
  }
}

void ElevatorEnteringTaskBase::ElevatorSimulator::LogRCG()
{
  if (mTracker->mRCGLogger) {
    for (int i = 0; i <= HumanTracker::LOG_ALL; ++i) {
      mTracker->mRCGLogger[i]->LogRectangular(
          mElevatorLeft - mWallWidth, mElevatorLeft,
          mEnvironmentTop, mElevatorTop, mWallColor);
      mTracker->mRCGLogger[i]->LogRectangular(
          mElevatorLeft - mWallWidth, mElevatorLeft,
          mElevatorBottom, mEnvironmentBottom, mWallColor);
      mTracker->mRCGLogger[i]->LogRectangular(
          mElevatorLeft, mElevatorRight,
          mElevatorTop - mWallWidth, mElevatorTop, mWallColor);
      mTracker->mRCGLogger[i]->LogRectangular(
          mElevatorLeft, mElevatorRight,
          mElevatorBottom, mElevatorBottom + mWallWidth, mWallColor);
      mTracker->mRCGLogger[i]->LogRectangular(
          mElevatorRight, mElevatorRight + mWallWidth,
          mElevatorTop, mElevatorBottom, mWallColor);
      mTracker->mRCGLogger[i]->LogRectangular(
          mElevatorLeft - mWallWidth,
          mElevatorLeft, mElevatorTop, (mElevatorTop + mElevatorBottom) / 2,
          mWallColor);
      mTracker->mRCGLogger[i]->LogRectangular(
          mElevatorLeft + mWallWidth, LeftInnerBoundary(),
          mElevatorTop, (mElevatorTop + mElevatorBottom) / 2, mWallColor);
      mTracker->mRCGLogger[i]->LogRectangular(
          DoorLeft(), DoorRight(), DoorTop(), DoorBottom(), mWallColor);
    }
  }
}

ElevatorEnteringTask_SL::ElevatorEnteringTask_SL():
    ElevatorEnteringTaskBase("ElevatorEnteringTask_SL")
{

}

ElevatorEnteringTask_SL::~ElevatorEnteringTask_SL()
{
}

void ElevatorEnteringTask_SL::RegisterIntentions()
{
  IntentionFactory::ins().Register(make_shared<ElevatorEnteringTaskBase::Stay>(GetTracker()));
  IntentionFactory::ins().Register(make_shared<ElevatorEnteringTaskBase::Leave>(GetTracker()));
}

ElevatorEnteringTask_SLH::ElevatorEnteringTask_SLH():
    ElevatorEnteringTaskBase("ElevatorEnteringTask_SLH")
{

}

ElevatorEnteringTask_SLH::~ElevatorEnteringTask_SLH()
{
}

void ElevatorEnteringTask_SLH::RegisterIntentions()
{
  IntentionFactory::ins().Register(make_shared<ElevatorEnteringTaskBase::Stay>(GetTracker()));
  IntentionFactory::ins().Register(make_shared<ElevatorEnteringTaskBase::Leave>(GetTracker()));
  IntentionFactory::ins().Register(make_shared<ElevatorEnteringTaskBase::Hesitate>(GetTracker()));
}

