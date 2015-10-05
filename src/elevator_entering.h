/*
 * elevator.h
 *
 *  Created on: Jan 10, 2014
 *      Author: baj
 */

#ifndef ELEVATOR_H_
#define ELEVATOR_H_

#include "task.h"
#include "intention.h"
#include "simulator.h"

class HumanTracker;

class ElevatorEnteringTaskBase: public Task
{
public:
  ElevatorEnteringTaskBase();
  ElevatorEnteringTaskBase(std::string name);
  virtual ~ElevatorEnteringTaskBase();

  virtual vector2d NewHumanPosition();
  virtual shared_ptr<Simulator> CreateSimulator();

public:
  class Stay: public HumanIntention {
  public:
    Stay(HumanTracker *tracker);
    virtual ~Stay();

    virtual shared_ptr<HumanAction> ChooseAction(const HumanState &hs);
  };

  class Leave: public HumanIntention {
  public:
    Leave(HumanTracker *tracker);
    virtual ~Leave();

    virtual shared_ptr<HumanAction> ChooseAction(const HumanState &hs);
  };

  class Hesitate: public HumanIntention {
  public:
    Hesitate(HumanTracker *tracker);
    virtual ~Hesitate();

    virtual shared_ptr<HumanAction> ChooseAction(const HumanState &hs);
    virtual HumanIntention *ChangeIntention(const HumanState &hs, double duration);
    virtual HashMap<HumanIntention*, double> GetTransitions(const HumanState &hs, double duration);
  };

  class ElevatorSimulator: public Simulator {
  public:
    ElevatorSimulator(Task *task, HumanTracker *tracker);
    virtual ~ElevatorSimulator();

    virtual void Initialize(double expected_humans);
    virtual void Simulate(double dt, double birth_rate, double death_rate);
    virtual bool End();
    virtual void CheckConsistence(Observation &obs);
    virtual void LogRCG();

  private:
    double DoorTop() const {
      return mDoorPosition;
    }

    double DoorBottom() const {
      return mDoorPosition + (mElevatorBottom - mElevatorTop) / 2;
    }

    bool EdgeCrossesWalls(const vector2d& v1, const vector2d& v2) const {
      for(unsigned int i=0; i< mWalls.size(); i++) {
        if(mWalls[i].intersects(v1, v2, false, false, true))
          return true;
      }
      return false;
    }

    bool EdgeCrossesDoor(const vector2d& v1, const vector2d& v2) const {
      line2d door(vector2d(DoorRight(), DoorTop()),
                  vector2d(DoorRight(), DoorBottom()));
      return door.intersects(v1, v2, false, false, true);
    }

    bool InFieldOfView(const vector2d& v) {
      if (v.x > mElevatorLeft) {
        if (EdgeCrossesWalls(v, mCenter) || EdgeCrossesDoor(v, mCenter)) {
          return false;
        }

        double dir1 = (vector2d(DoorLeft(), DoorBottom()) -
            mTracker->GetRobotPose().mPosition).angle();
        double dir2 = (vector2d(mElevatorLeft - mWallWidth, mElevatorBottom) -
            mTracker->GetRobotPose().mPosition).angle();
        double dir = (v - mTracker->GetRobotPose().mPosition).angle();

        return _angle::IsAngleRadInBetween(dir1, dir, dir2) &&
            mTracker->InFieldOfView(v);
      }
      else {
        return mTracker->InFieldOfView(v);
      }
    }

  private:
    uint mHumanCount;
    double mDoorPosition;
    double mDoorDirection;
    double mDoorOpenTime;
    double mDoorCloseTime;

    std::vector<line2d> mWalls;
    vector2d mCenter;

    static const double mExpectedDoorOpenTime;
    static const QColor mWallColor;
  };

public:
  static double LeftInnerBoundary() {
    return mElevatorLeft + 2.0 * mWallWidth;
  }

  static double LeftOuterBoundary() {
    return mElevatorLeft - mWallWidth;
  }

  static double DoorLeft() {
    return mElevatorLeft;
  }

  static double DoorRight() {
    return mElevatorLeft + mWallWidth;
  }

  static const double mElevatorLeft;
  static const double mElevatorRight;
  static const double mElevatorTop;
  static const double mElevatorBottom;
  static const double mEnvironmentTop;
  static const double mEnvironmentBottom;
  static const double mWallWidth;
};

class ElevatorEnteringTask_SL: public ElevatorEnteringTaskBase {
public:
  ElevatorEnteringTask_SL();
  virtual ~ElevatorEnteringTask_SL();

  virtual void RegisterIntentions();
};

class ElevatorEnteringTask_SLH: public ElevatorEnteringTaskBase {
public:
  ElevatorEnteringTask_SLH();
  virtual ~ElevatorEnteringTask_SLH();

  virtual void RegisterIntentions();
};


#endif /* ELEVATOR_H_ */
