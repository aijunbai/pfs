/*
 * state.h
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#ifndef STATE_H_
#define STATE_H_

#include "geometry.h"
#include "logger.h"
#include "common.h"
#include "intention.h"
#include "observation.h"

class IdentifiedHuman;

class HumanState
{
private:
  const HumanState &operator=(const HumanState &o);

public:
  typedef shared_ptr<HumanState> Ptr;

public:
  explicit HumanState(Detection::Ptr det = Detection::Ptr());
  explicit HumanState(const HumanState &o);

  virtual ~HumanState();

  void CopyFrom(const HumanState &o);

  void IntentionAwarePredict(double duration, bool transition = true);

  void AddNoise();
  void SetIntention(HumanIntention *intention);

  int Age();

  vector2d &Position() {
    return mPosition;
  }

  const vector2d &Position() const {
    return mPosition;
  }

  void SetPosition(const vector2d &pos) {
    mPosition = pos;
  }

  vector2d &Velocity() {
    return mVelocity;
  }

  const vector2d &Velocity() const {
    return mVelocity;
  }

  void SetVelocity(const vector2d &vel) {
    mVelocity = vel;
  }

  void SetDetection(Detection::Ptr &det);

  double & Orientation() {
    return mOrientation;
  }

  const double & Orientation() const {
    return mOrientation;
  }

  void SetOrientation(double orient) {
    mOrientation = orient;

    if (isinf(mOrientation) || isnan(mOrientation)) {
      mOrientation = DBL_MAX;
    }
  }
  HumanIntention *Intention() const {
    return mIntentionDistri.Sampling();
  }

//  RCGLogger::Color IntentionColor() const;
  QColor Color() const;

  void Log(
      RCGLogger::Ptr &logger,
      bool velocity = true,
      const char *label = 0);

public:
  const uint mID; //unique id

  uint mBornTime;
  vector2d mPosition;
  vector2d mVelocity;
  double mOrientation;

  IdentifiedHuman *mIdentity;
  shared_ptr<TimedDetection> mDetection;
  IntentionDistribution mIntentionDistri;

public:
  friend std::ostream& operator<<(std::ostream &os, const HumanState &o) {
    return os << "(id=" << o.mID
        << ", born=" << o.mBornTime
        << " pos=" << o.Position().x
        << "," << o.Position().y << ")";
  }
};

#endif /* STATE_H_ */
