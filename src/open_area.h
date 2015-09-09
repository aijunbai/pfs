/*
 * open_area.h
 *
 *  Created on: Feb 13, 2014
 *      Author: baj
 */

#ifndef OPEN_AREA_H_
#define OPEN_AREA_H_

#include "task.h"
#include "intention.h"
#include "simulator.h"

class HumanTracker;

class OpenAreaTask: public Task
{
public:
  OpenAreaTask();
  virtual ~OpenAreaTask();

  virtual void RegisterIntentions();
  virtual vector2d NewHumanPosition();
  virtual shared_ptr<Simulator> CreateSimulator();

public:
  class Stay: public HumanIntention {
  public:
    Stay(HumanTracker *tracker);
    virtual ~Stay();

    virtual shared_ptr<HumanAction> ChooseAction(const HumanState &hs);
  };

  class Move: public HumanIntention {
  public:
    Move(HumanTracker *tracker);
    virtual ~Move();

    virtual shared_ptr<HumanAction> ChooseAction(const HumanState &hs);
  };

  class OpenAreaSimulator: public Simulator {
  public:
    OpenAreaSimulator(Task *task, HumanTracker *tracker);
    virtual ~OpenAreaSimulator();

    virtual void Initialize(double expected_humans);
    virtual void Simulate(double duration, double birth_rate, double death_rate);
    virtual bool End();
    virtual void CheckConsistence(Observation &obs);
    virtual void LogRCG();

    bool InFieldOfView(const vector2d& v) {
      return mTracker->InFieldOfView(v);
    }
  };

public:
  static const double mLeft;
  static const double mRight;
  static const double mTop;
  static const double mBottom;
};

#endif /* OPEN_AREA_H_ */
