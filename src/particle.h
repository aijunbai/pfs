/*
 * particle.h
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <vector>
#include <list>
#include <cassert>
#include <algorithm>
#include <boost/enable_shared_from_this.hpp>
#include <boost/tuple/tuple.hpp>

#include "state.h"
#include "observation.h"
#include "logger.h"
#include "combination.h"
#include "statistic.h"
#include "params.h"

class Task;
class HumanTracker;

class Particle: public boost::enable_shared_from_this<Particle>
{
public:
  typedef shared_ptr<Particle> Ptr;

  template <class T>
  struct Element: public T {
    typedef shared_ptr<Element<T> > Ptr;

    explicit Element(double prob1, double prob2):
      prob1(prob1), prob2(prob2)
    {

    }

    bool operator>(const Element<T> &o) const
    {
      return prob() > o.prob();
    }

    double prob() const {
      return prob1 * prob2;
    }

    double prob1;
    double prob2;
  };

  template <class T>
  struct PtrGreater {
    bool operator() (const typename T::Ptr &a, const typename T::Ptr &b) {
      return *a > *b;
    }
  };

  template <class T>
  struct PtrLesser {
    bool operator() (const typename T::Ptr &a, const typename T::Ptr &b) {
      return *a < *b;
    }
  };

  template <class T>
  struct Results: public std::vector<shared_ptr<T> > {
    typedef shared_ptr<Results<T> > Ptr;

    explicit Results(double total_prob):
      total_prob(total_prob)
    {

    }

    shared_ptr<T> propose()
    {
      assert(!this->empty());
      return Params::ins().assignment_sampling? sampling(): best();
    }

    shared_ptr<T> best()
    {
      return this->empty()? shared_ptr<T>(): this->front();
    }

    shared_ptr<T> sampling()
    {
      if (this->size() <= 1) {
        return this->empty()? shared_ptr<T>(): this->front();
      }

      std::vector<double> cdf(this->size() + 1, 0.0);
      for (uint i = 0; i < this->size(); ++i) {
        cdf[i+1] = cdf[i] + this->at(i)->prob();
      }

      foreach_(double &f, cdf) {
        f /= cdf.back();
      }
      cdf.back() = 1.0;

      uint i = std::distance(cdf.begin(), std::upper_bound(
          cdf.begin(), cdf.end(), SimpleRNG::ins().GetUniform())) - 1;

      return this->at(i % this->size());
    }

  public:
    double total_prob;
  };

  typedef Element<std::vector<uint> > Permutation;
  typedef Element<Combination::Table<uint> > Assignment;
  typedef Results<Permutation> Permutations;
  typedef Results<Assignment> Assignments;

  class HumanList: public std::vector<HumanState::Ptr> {
  private:
    reference operator[](size_type __n);

  public:
    virtual ~HumanList() {

    }

    HumanState *get(int i) {
      if (i < 0 || i >= int(size())) {
        assert(0);
        return 0;
      }

      return std::vector<HumanState::Ptr>::at(i).get();
    }

    iterator erase(iterator __position) {
      return std::vector<HumanState::Ptr>::erase(__position);
    }

    bool push_back(const value_type &__x) {
      if (size() < Params::ins().max_humans) {
        std::vector<HumanState::Ptr>::push_back(__x);
        return true;
      }

      return false;
    }
  };

private:
  const Particle &operator=(const Particle &o);

public:
  explicit Particle();
  explicit Particle(const Particle &o);
  virtual ~Particle();

  void CopyFrom(const Particle &o);

  void Predict(double duration);
  Particle::Ptr Update(CachedObservation &obs, double duration);
  Particle::Ptr Refine(CachedObservation &obs, double duration);

  void ComputeObservationProbImp(
      CachedObservation &obs, double duration, bool pruning);

  void ComputeObservationProb(
      CachedObservation &obs, double duration);

  double &Likelihood() {
    assert(mAssignments);
    assert(mAssignments->total_prob > 0.0);
    return mAssignments->total_prob;
  }

private:
  void Reset(CachedObservation &obs);

  Permutations::Ptr ComputeTruePositiveProb(
      Combination::Table<int> &obs,
      Combination::Table<int> &hs,
      bool pruning);

  Permutations::Ptr ComputeTruePositiveProbImp(
      Combination::Table<int> &obs,
      Combination::Table<int> &hs,
      bool pruning);

  Permutations::Ptr ComputeFullTruePositiveProb(
      Combination::Table<int> &obs,
      Combination::Table<int> &hs);

  void EnumerateAllPermutations(
      Permutations &perms,
      Permutation &perm,
      Combination::Table<int> &obs,
      Combination::Table<int> &hs,
      int i = 0);

public:
  void Log(RCGLogger::Ptr &logger,
           bool verbose = true,
           const char *label = 0);

public:
  HumanList mHumans;
  Assignments::Ptr mAssignments;

  HashMap<int, HashMap<int, double> > mProbCache;

  static const uint mMurtyThreshold;
  static const double mProbThreshold;

  static STATISTIC::Ptr mComparedPairsStat[2];
  static STATISTIC::Ptr mPushedPairsStat[2];
  static STATISTIC::Ptr mAllPairsStat[2];
  static STATISTIC::Ptr mComputedPermutationsStat[2];
  static STATISTIC::Ptr mAllPermutationsStat[2];
  static STATISTIC::Ptr mComputedCombinationStat[2];
  static STATISTIC::Ptr mAllCombinationsStat[2];
  static STATISTIC::Ptr mComputedTermsStat[2];
  static STATISTIC::Ptr mAllTermsStat[2];
  static STATISTIC::Ptr mTimeUsageStat[2];

  static STATISTIC::Ptr mAssignmentApproximateError;
  static STATISTIC::Ptr mFalseMissingApproximateError;

  static void CreateStatistics();
  static void PrintStatistics();

private:
  static void CreateStatistics(bool pruning);
  static void PrintStatistics(bool pruning);
};

#endif /* PARTICLE_H_ */
