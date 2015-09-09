/*
 * action.h
 *
 *  Created on: Jan 10, 2014
 *      Author: baj
 */

#ifndef ACTION_H_
#define ACTION_H_

#include <string>
#include "params.h"
#include "common.h"

class HumanState;

class HumanAction
{
public:
  HumanAction(const char *name);
  virtual ~HumanAction();

  virtual void Apply(HumanState &hs, double duration) = 0;

  const char *GetName() {
    return mName;
  }

private:
  const char *mName;
};

namespace _motion
{

class Stay: public HumanAction
{
public:
  Stay();
  virtual ~Stay();

  virtual void Apply(HumanState &hs, double duration);
};

class Dash: public HumanAction
{
public:
  Dash(double power, double direction);
  virtual ~Dash();

  virtual void Apply(HumanState &hs, double duration);

private:
  double mPower;
  double mAngle;
};

}

#endif /* ACTION_H_ */
