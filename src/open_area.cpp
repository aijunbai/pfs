/*
 * open_area.cpp
 *
 *  Created on: Feb 13, 2014
 *      Author: baj
 */

#include <boost/math/distributions/negative_binomial.hpp>
#include <boost/math/distributions/normal.hpp>
#include <open_area.h>
#include "params.h"

using namespace std;

const double OpenAreaTask::mLeft = -7.5;
const double OpenAreaTask::mRight = 7.5;
const double OpenAreaTask::mTop = -3.8;
const double OpenAreaTask::mBottom = 3.8;

namespace {
bool ret = Task::Register<OpenAreaTask>("OpenAreaTask");
}

OpenAreaTask::OpenAreaTask(): Task("OpenAreaTask")
{

}

OpenAreaTask::~OpenAreaTask()
{
}

void OpenAreaTask::RegisterIntentions()
{
  IntentionFactory::ins().Register(make_shared<OpenAreaTask::Stay>(GetTracker()));
  IntentionFactory::ins().Register(make_shared<OpenAreaTask::Move>(GetTracker()));
}

vector2d OpenAreaTask::NewHumanPosition()
{
  return _motion::RandomPosition(mLeft, mRight, mTop, mBottom);
}

shared_ptr<Simulator> OpenAreaTask::CreateSimulator()
{
  return make_shared<OpenAreaSimulator>(this, GetTracker());
}

OpenAreaTask::Stay::Stay(HumanTracker *tracker):
    HumanIntention("OpenAreaTask::Stay", tracker)
{

}

OpenAreaTask::Stay::~Stay()
{
}

shared_ptr<HumanAction> OpenAreaTask::Stay::ChooseAction(
    const HumanState &hs)
{
  return make_shared<_motion::Stay>();
}

OpenAreaTask::Move::Move(HumanTracker *tracker):
    HumanIntention("OpenAreaTask::Move", tracker)
{

}

OpenAreaTask::Move::~Move()
{
}

shared_ptr<HumanAction> OpenAreaTask::Move::ChooseAction(
    const HumanState &hs)
{
  double power = _motion::RandomPower();
  double angle = _motion::RandomAngle();

  return make_shared<_motion::Dash>(power, angle);
}

OpenAreaTask::OpenAreaSimulator::OpenAreaSimulator(
    Task *task, HumanTracker *tracker):
    Simulator(task, tracker)
{
  Params::ins().false_rate = 1.0;
  Params::ins().missing_rate = 1.0;
  Params::ins().death_rate = 0.02;
  Params::ins().observation_error = 0.25;

  Params::ins().view_width = 360.0;
  Params::ins().max_humans = 10;
}

OpenAreaTask::OpenAreaSimulator::~OpenAreaSimulator()
{
}

void OpenAreaTask::OpenAreaSimulator::Initialize(double expected_humans)
{
  RobotPose pose;
  pose.mPosition = vector2d(0.0, 0.0);
  pose.mAngle = RAD(180.0);

  mTracker->UpdateRobotPose(pose);

  mHumans.clear();

  for (uint i = 0; i < expected_humans; ++i) {
    vector2d initial_pos = _motion::RandomPosition(mLeft, mRight, mTop, mBottom);

    mHumans.push_back(make_shared<Simulator::Human>(
        initial_pos, initial_pos,
        IntentionFactory::ins().GetRandomIntention()));
  }
}

void OpenAreaTask::OpenAreaSimulator::CheckConsistence(Observation &obs)
{

}

bool OpenAreaTask::OpenAreaSimulator::End()
{
  return false;
}

void OpenAreaTask::OpenAreaSimulator::Simulate(
    double duration, double birth_rate, double death_rate)
{
  if (death_rate > 0.0 && !mHumans.empty()) {
    const int m = SimpleRNG::ins().GetPoisson(death_rate * mHumans.size() * duration);

    for (int i = 0; i < m; ++i) {
      list<Human::Ptr>::iterator it = mHumans.begin();
      advance(it, SimpleRNG::ins().GetRand() % mHumans.size());
      mHumans.erase(it);

      if (mHumans.empty()) break;
    }
  }

  if (birth_rate > 0.0) {
    const int n = SimpleRNG::ins().GetPoisson(birth_rate * duration);

    for (int i = 0; i < n; ++i) {
      vector2d initial_pos = _motion::RandomPosition(mLeft, mRight, mTop, mBottom);
      mHumans.push_back(make_shared<Simulator::Human>(
          initial_pos, initial_pos,
          IntentionFactory::ins().GetRandomIntention()));
    }
  }

  foreach_(Human::Ptr &human, mHumans) {
    if (SimpleRNG::ins().GetUniform() < 0.01) {
      HumanIntention *intention = IntentionFactory::ins().GetRandomIntention();
      human->SetIntention(intention);
    }

    HumanIntention *intention = human->Intention();

    if (intention && intention->GetName() == "OpenAreaTask::Move") {
      if (human->TargetDist() < 1.0) {
        human->mTarget = _motion::RandomPosition(
            mLeft, mRight, mTop, mBottom);
      }

      double angle = human->TargetAngle();
      double power = 0.5;

      _motion::Dash(power, angle).Apply(*human, duration);
    }
    else {
      human->Predict(duration);
    }
  }

  RobotPose pose = mTracker->GetRobotPose();
  pose.mAngle += RAD(1.0);

  mTracker->UpdateRobotPose(pose);
}

void OpenAreaTask::OpenAreaSimulator::LogRCG()
{

}
