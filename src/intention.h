/*
 * intention.h
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#ifndef INTENTION_H_
#define INTENTION_H_

#include <string>
#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "action.h"
#include "common.h"
#include "logger.h"

class HumanState;
class HumanTracker;

class HumanIntention
{
public:
  HumanIntention(std::string name, HumanTracker *tracker);
  virtual ~HumanIntention();

  std::string &GetName() {
    return mName;
  }

  HumanTracker *GetTracker() {
    return mTracker;
  }

  virtual shared_ptr<HumanAction> ChooseAction(const HumanState &hs) = 0;

  virtual HumanIntention *ChangeIntention(const HumanState &hs, double duration) {
    return this;
  }

  virtual HashMap<HumanIntention*, double> GetTransitions(const HumanState &hs, double duration) {
    HashMap<HumanIntention*, double> trans;
    trans[this] = 1.0;

    return trans;
  }

public:
  const uint mID;

//  RCGLogger::Color Color() {
//    return RCGLogger::Color(mID % RCGLogger::Color_Max);
//  }

private:
  std::string mName;
  HumanTracker *mTracker;

  static uint mIDCounter;
};

class IntentionFactory
{
private:
  IntentionFactory();
  virtual ~IntentionFactory();

public:
  static IntentionFactory &ins();

  void Register(shared_ptr<HumanIntention> intention);
  HumanIntention *GetIntention(const std::string &name);
  HumanIntention *GetIntention(uint iid) const;
  HumanIntention *GetRandomIntention() const;
  const std::vector<shared_ptr<HumanIntention> > &GetIntentions() const;
  Eigen::MatrixXd GetTransitionMatrix(const HumanState &hs, double duration);

  uint Size() const {
    return mIntentions.size();
  }

private:
  HashMap<std::string, shared_ptr<HumanIntention> > mIntentionMap;
  std::vector<shared_ptr<HumanIntention> > mIntentions;
};

typedef Eigen::VectorXd IntentionVector;

class IntentionDistribution: public IntentionVector {
public:
  IntentionDistribution(): IntentionVector(IntentionVector::Zero(
      IntentionFactory::ins().Size())) {
    assert(IntentionFactory::ins().Size());
  }

  IntentionDistribution(const IntentionVector &o): IntentionVector(o) {

  }

  IntentionDistribution(const IntentionDistribution &o): IntentionVector(o) {

  }

  virtual ~IntentionDistribution() {

  }

public:
  typedef IntentionDistribution (*Initializer)();

  static Initializer mInitializers[3];

  static IntentionDistribution Random() {
    return mInitializers[Max(Params::ins().intention_mode, 0)]();
  }

  static IntentionDistribution RandomVertex() {
    IntentionDistribution ret;
    assert(IntentionFactory::ins().Size() == ret.size());

    ret[SimpleRNG::ins().GetRand() % ret.size()] = 1.0;

    ret.Assertion();
    return ret;
  }

  static IntentionDistribution RandomSimplex() {
    IntentionDistribution ret;
    assert(IntentionFactory::ins().Size() == ret.size());

    for (int i = 0; i < ret.size(); ++i) {
      ret[i] = SimpleRNG::ins().GetExponential(1.0);
    }

    ret /= ret.sum();

    ret.Assertion();
    return ret;
  }

  static IntentionDistribution RandomUniform() {
    IntentionDistribution ret;
    assert(IntentionFactory::ins().Size() == ret.size());

    ret.setConstant(1.0 / ret.size());

    ret.Assertion();
    return ret;
  }

  void Assertion() const {
    assert(!isZero());
    assert(IntentionFactory::ins().Size() == size());
    assert(fabs(sum() - 1.0) < 1.0e-6);
  }

  HumanIntention *Sampling() const {
    Assertion();

    int id = -1;

    if (maxCoeff(&id) < 1.0) {
      std::vector<double> cmf(size() + 1, 0.0);

      for (int i = 0; i < size(); ++i) {
        cmf[i+1] = cmf[i] + this->operator [](i);
      }

      if (cmf.back() != 1.0) {
        foreach_(double &d, cmf) {
          d /= cmf.back();
        }
        cmf.back() = 1.0;
      }

      id = distance(cmf.begin(), upper_bound(
          cmf.begin(), cmf.end(), SimpleRNG::ins().GetUniform())) - 1;
    }

    return IntentionFactory::ins().GetIntention(id % size());
  }
};

#endif /* INTENTION_H_ */
