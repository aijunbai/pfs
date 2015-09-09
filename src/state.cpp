/*
 * state.cpp
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/poisson.hpp>
#include <sstream>

#include <state.h>
#include <intention.h>
#include "tracker.h"
#include "params.h"
#include "elevator_entering.h"
#include "simulator.h"

using namespace std;

HumanState::HumanState(Detection::Ptr det):
    mID(HumanTracker::NextID()),
    mBornTime(HumanTracker::mCurrentStep),
    mPosition(det? det->mPosition: vector2d(0.0, 0.0)),
    mVelocity(_motion::RandomVelocity()),
    mIdentity(0),
    mIntentionDistri(IntentionDistribution::Random())
{
  SetDetection(det);

  mIntentionDistri.Assertion();
}

HumanState::HumanState(const HumanState &o):
    mID(HumanTracker::NextID()),
    mBornTime(o.mBornTime),
    mPosition(o.mPosition),
    mVelocity(o.mVelocity),
    mIdentity(o.mIdentity),
    mDetection(o.mDetection),
    mIntentionDistri(o.mIntentionDistri)
{
  mIntentionDistri.Assertion();
}

HumanState::~HumanState()
{
}

void HumanState::SetIntention(HumanIntention *intention)
{
  if (intention) {
    uint iid = intention->mID;

    if (mIntentionDistri[iid] != 1.0) {
      mIntentionDistri.setZero();
      mIntentionDistri[iid] = 1.0;
    }

    mIntentionDistri.Assertion();
  }
}

void HumanState::CopyFrom(const HumanState &o)
{
  mBornTime = o.mBornTime;
  mPosition = o.mPosition;
  mVelocity = o.mVelocity;
  mIntentionDistri = o.mIntentionDistri;
  mDetection = o.mDetection;

  o.mIntentionDistri.Assertion();
  mIntentionDistri.Assertion();
}

void HumanState::IntentionAwarePredict(double duration, bool transition)
{
  mIntentionDistri.Assertion();

  HumanIntention *intention = 0;

  if (transition) {
    if (Params::ins().mixed_filters) {
      mIntentionDistri = IntentionVector(
          IntentionFactory::ins().GetTransitionMatrix(
              *this, duration) * mIntentionDistri);
      mIntentionDistri.Assertion();

      intention = Intention();
    }
    else {
      intention = Intention();
      intention = intention->ChangeIntention(*this, duration);

      SetIntention(intention);
    }
  }
  else {
    intention = Intention();
  }

  intention->ChooseAction(*this)->Apply(*this, duration);
}

RCGLogger::Color HumanState::Color() const
{
  if (mIntentionDistri.isZero()) {
    return RCGLogger::Black;
  }

  HumanIntention *intention = Intention();
  return intention->Color();
}

void HumanState::Log(
    RCGLogger::Ptr &logger, bool verbose, const char *label)
{
  if (verbose) {
    vector2d vel = Position() + 0.1 * Velocity();
    logger->AddLine(Position().x, Position().y, vel.x, vel.y, Color());
  }

  if (label) {
    std::stringstream ss;
    ss << label << " " << mID;
    logger->AddPoint(Position().x, Position().y, Color(), ss.str().c_str());
  }
  else {
    logger->AddPoint(Position().x, Position().y, Color());
  }
}

int HumanState::Age()
{
  return HumanTracker::mCurrentStep - mBornTime;
}

void HumanState::AddNoise()
{
  if (SimpleRNG::ins().GetUniform() < 0.05) {
    mIntentionDistri = IntentionDistribution::RandomVertex();
  }

  if (SimpleRNG::ins().GetUniform() < 0.05) {
    SetVelocity(_motion::RandomVelocity());
  }

  mPosition += _motion::NoiseVector();
  mVelocity += _motion::NoiseVector();
}

void HumanState::SetDetection(Detection::Ptr &det)
{
  if (det) {
    mDetection = make_shared<TimedDetection>(
        det, HumanTracker::mCumulativeDuration);
  }
}
