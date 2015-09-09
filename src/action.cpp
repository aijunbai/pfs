/*
 * action.cpp
 *
 *  Created on: Jan 10, 2014
 *      Author: baj
 */


#include <action.h>
#include "params.h"
#include "tracker.h"

HumanAction::HumanAction(const char *name): mName(name)
{

}

HumanAction::~HumanAction()
{
}

namespace _motion
{

Stay::Stay(): HumanAction("motion_model::Stay")
{

}

Stay::~Stay()
{
}

void Stay::Apply(HumanState &hs, double duration)
{
  hs.mVelocity.setAll(0.0);
}

Dash::Dash(double power, double direction):
    HumanAction("motion_model::Dash"),
    mPower(power),
    mAngle(direction)
{

}

Dash::~Dash()
{
}

void Dash::Apply(HumanState &hs, double duration)
{
  double power = Params::ins().sigma_power * mPower;

  vector2d acc = vector2d(power, 0.0).rotate(
      mAngle + _motion::NoiseAngle()) + _motion::NoiseVector();

  hs.mPosition += hs.mVelocity * duration + 0.5 * acc * duration * duration;
  hs.mVelocity += acc * duration;

  if (hs.mVelocity.length() > Params::ins().max_speed) {
    hs.mVelocity *= Params::ins().max_speed / hs.mVelocity.length();
  }
}

}
