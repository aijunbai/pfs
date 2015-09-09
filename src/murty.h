/*
 * murty.h
 *
 *  Created on: Jan 20, 2014
 *      Author: baj
 */

#ifndef MURTY_H_
#define MURTY_H_

#include <stdio.h>
#include <string>

#include "common.h"
#include "statistic.h"
#include "combination.h"
#include "particle.h"
#include "simulator.h"

class Murty
{
private:
  Murty(const char *path);
  virtual ~Murty();

public:
  static Murty &ins();

  Particle::Permutations::Ptr Solve(
      HashMap<int, HashMap<int, double> > &prob_mat,
      Combination::Table<int> &obs_set,
      Combination::Table<int> &hs_set);

  Particle::Permutation::Ptr Solve(
      HashMap<int, HashMap<int, double> > &prob_mat,
      uint cols,
      Particle &particle);

  Particle::Permutation::Ptr Solve(
      HashMap<int, HashMap<int, double> > &o_prob,
      uint rows,
      std::vector<IdentifiedHuman::Ptr> &identified_humans);

  Particle::Permutation::Ptr Solve(
      std::vector<Simulator::Human::Ptr> &objects,
      std::vector<IdentifiedHuman::Ptr> &hypothese);

  static void CreateStatistics();
  static void PrintStatistics();

private:
  static STATISTIC::Ptr mTimeUsageStat;
  static STATISTIC::Ptr mMurtySizeStat;

private:
  Particle::Permutations::Ptr SolveImp(std::vector<double> &buf);
  Particle::Permutations::Ptr CallMiller(std::vector<double> &buf);
};

#include <eigen3/Eigen/Core>

#define __AUCTION_EPSILON_MULTIPLIER 1e-5
#define __AUCTION_INF 1e8
#define __AUCTION_ZERO 0.

//#define __AUCTION_OMIT_ZEROS


/**
 * represents an undirected edge between node x and y with weight v
 */
template<typename Scalar = double>
struct Edge
{
public:
  typedef shared_ptr<Edge<Scalar> > Ptr;

  Edge() : x(0), y(0), v(0) {}
  Edge(const size_t x, const size_t y, const Scalar v) :
    x(x), y(y), v(v) {}

  size_t x;
  size_t y;
  Scalar v;

  // sort edges by row
  bool operator<(const Edge<Scalar> & o) const {
    return x < o.x;
  }
};

/**
 * vector of edges
 */
template <typename Scalar = double>
struct Edges: public std::vector<typename Edge<Scalar>::Ptr > {
  typedef shared_ptr<Edges> Ptr;

};

template<typename Scalar = double>
class Auction
{
private:

  Auction();
  virtual ~Auction() {

  }

public:

  typedef Eigen::Matrix<Scalar, -1, -1> WeightMatrix;

  /**
   * vector of scalars (prices, profits, ...)
   */
  typedef std::vector<Scalar> Scalars;

  /**
   * vector of bools for row/column-locking
   */
  typedef std::vector<bool> Locks;

  /**
   * vector of indices
   */
  typedef std::vector<size_t> indices;

  static const typename Edges<Scalar>::Ptr solve(const Eigen::Matrix<Scalar, -1, -1> & a)
  {
    const size_t rows = a.rows();
    const size_t cols = a.cols();

    Locks lockedRows(a.rows(), false);
    Locks lockedCols(a.cols(), false);
    typename Edges<Scalar>::Ptr E = make_shared<Edges<Scalar> >();

    Scalar lambda = .0;
    Scalar epsilon = __AUCTION_EPSILON_MULTIPLIER / a.cols();

    // condition 3: initially set p_j >= lambda
    Scalars prices(cols, 0.), profits(rows, 1.); // p-Vector  (1 to j) = p_j

    do
    {
      //    Step 1 (forward auction cycle):
      //    Execute iterations of the forward auction algorithm until at least one
      //    more person becomes assigned. If there is an unassigned person left, go
      //    to step 2; else go to step 3.
      while (forward(a, E, prices, profits, lockedRows, lockedCols,
                     lambda, epsilon))
        ;

      if (!allPersonsAssigned(lockedRows))
      {

        //    Step 2 (reverse auction cycle):
        //    Execute several iterations of the reverse auction algorithm until at least
        //    one more object becomes assigned or until we have p_j <= lambda for all
        //    unassigned objects. If there is an unassigned person left, go to step 1
        //    else go to step 3
        while (!reverse(a, E, prices, profits, lockedRows, lockedCols,
                        lambda, epsilon)
            || !unassignedObjectsLTlambda(lockedCols, prices,
                                          lambda))
          ; // reverse auction
      }

      if (allPersonsAssigned(lockedRows))
      {
        //    Step 3 (reverse auction):
        //    Execute successive iterations of the reverse auction algorithm until the
        //    algorithm terminates with p_j <= lambda for all unassigned objects j
        while (true)
        {
          reverse(a, E, prices, profits, lockedRows, lockedCols,
                  lambda, epsilon);
          if (unassignedObjectsLTlambda(lockedCols, prices, lambda))
            break;
        }
        break;
      }

    } while (true);

    return E;
  }
private:

  /**
   * forward cycle of auction algorithm
   * @param a weight matrix (nxm)
   * @param S assignment matrix (nxm)
   * @param prices prices per object (m)
   * @param profits profits per person (n)
   * @param lambda bidding threshold lambda
   * @param epsilon bidding increment
   * @return true if assignment was made, false otherwise
   */
  static bool forward(const Eigen::Matrix<Scalar, -1, -1> & a,
                      typename Edges<Scalar>::Ptr & E,
                      Scalars & prices, Scalars & profits, Locks & lockedRows,
                      Locks & lockedCols, Scalar & lambda, Scalar & epsilon)
  {
    const size_t rows = a.rows();
    const size_t cols = a.cols();
    bool assignmentFound = false;

    for (size_t i = 0; i < rows; i++) // for the i-th row/person
    {
      bool assignmentInThisIterationFound = false;

      // person already assigned?
      if (lockedRows[i])
        continue;

      // find an unassigned person i, its best object j_i
      // j_i = argmax {a_ij - p_j} for j in A(i) ( A(i) are the set of edges of the i-th row )
      // if a(i,j) = 0. it is not a valid edge
      size_t j_i = 0;

      //  v_i = max { a_ij - p_j} for j in A(i)       // maximum profit for person i
      //  v_i was already found = v_i
      //  w_i = max { a_ij - p_j} for j in A(i) and j != j_i  // second best profit
      //  if j_i is the only entry in A(i), w_i = - inf       // there's no second best profit
      Scalar w_i = -__AUCTION_INF, v_i = -__AUCTION_INF, a_i_ji = 0.; // = max { a_ij - p_j}

      // find maximum profit i.e. j_i = arg max { a_ij - p_j} and second best
      for (size_t j = 0; j < cols; j++) // for the j-th column
      {
        const Scalar aij = a(i,j);
#ifndef __AUCTION_OMIT_ZEROS
        if ( aij == __AUCTION_ZERO ) continue;
#endif
        const Scalar diff = aij - prices[j];
        if (diff > v_i)
        {
          // if there already was an entry found, this is the second best
          if (assignmentInThisIterationFound)
            w_i = v_i;

          v_i = diff;
          j_i = j;
          a_i_ji = aij;
          assignmentInThisIterationFound = true;
        }
        if (diff > w_i && j_i != j)
          w_i = diff;
        // if no entry is bigger than v_i, check if there's still a bigger second best entry
      }

      // no possible assignment found?
      if (!assignmentInThisIterationFound)
      {
        lockedRows[i] = true; // if no assignment found in this row, there is no arc ...
        continue;
      }
      assignmentInThisIterationFound = false;

      //      std::cout << "assignment found .." << std::endl;
      const Scalar bid = a_i_ji - w_i + epsilon;

      //  P_i = w_i - E
      profits[i] = w_i - epsilon; // set new profit for person

      //  prices(j_i) = max(lambda, a(i,j_i) - w(i) + epsilon)
      // if lambda <= a_ij - w_i + E, add (i, j_i) to S
      if (lambda <= bid)
      {
        prices[j_i] = bid;
        // assignment was made, so lock row and col
        lockedRows[i] = true;
        lockedCols[j_i] = true;

        bool newEdge = true;

        // if j_i was assigned to different i' to begin, remove (i', j_i) from S
        foreach_ (typename Edge<Scalar>::Ptr & e, *E) {
          if (e->y == j_i) // change edge
          {
            lockedRows[e->x] = false; // unlock row i'
            newEdge = false;
            e->x = i;
            e->v = a_i_ji;
            break;
          }
        }
        if (newEdge)
        {
          typename Edge<Scalar>::Ptr e = make_shared<Edge<Scalar> >();
          e->x = i;
          e->y = j_i;
          e->v = a_i_ji;
          E->push_back(e);
        }
        assignmentInThisIterationFound = true;

      }
      else
      {
        prices[j_i] = lambda;
        assignmentInThisIterationFound = false;

      }
      if (assignmentInThisIterationFound)
        assignmentFound = true;
    }
    return assignmentFound;

  }

  /**
   * reverse cycle of auction algorithm
   * @param a weight matrix (nxm)
   * @param S assignment matrix (nxm)
   * @param prices prices per object (m)
   * @param profits profits per person (n)
   * @param lambda bidding threshold lambda
   * @param epsilon bidding increment
   * @return true if assignment was made, false otherwise
   */
  static bool reverse(const Eigen::Matrix<Scalar, -1, -1> & a,
                      typename Edges<Scalar>::Ptr & E,
                      Scalars & prices, Scalars & profits, Locks & lockedRows,
                      Locks & lockedCols, Scalar & lambda, const Scalar & epsilon)
  {
    const size_t rows = a.rows();
    const size_t cols = a.cols();

    bool assignmentFound = false;

    for (size_t j = 0; j < cols; j++) // for the j-th column (objects)
    {
      bool assignmentInThisIterationFound = false;

      // object already assigned,  p_j > lambda ?
      if (lockedCols[j])
        continue;

      if (!(prices[j] > lambda))
        continue;

      // Find an unassigned object j with p_j > lambda, its best person i_j
      // i_j = argmax {a_ij - profits[i]) für i aus B(j) (PI !!!)
      size_t i_j = 0;

      //g_j = max {a_ij - P_i} for i in B(j) and i != i_j
      // if j_i is the only entry in B(j), g_j = - inf ( g_j < b_j)
      //b_j = max {a_ij - P_i} for i in B(j)
      Scalar b_j = -__AUCTION_INF, g_j = -__AUCTION_INF, a_ij_j = 0.;

      // find maximum profit i.e. j_i = arg max { a_ij - p_j} and second best
      for (size_t i = 0; i < rows; i++) // for the j-th column
      {
        const Scalar aij = a(i, j);
#ifndef __AUCTION_OMIT_ZEROS
        if ( aij == __AUCTION_ZERO ) continue;
#endif
        const Scalar diff = aij - profits[i];
        if (diff > b_j)
        {
          // if there already was an entry found, this is the second best
          if (assignmentInThisIterationFound)
            g_j = b_j;

          b_j = diff;
          i_j = i;
          a_ij_j = aij;
          assignmentInThisIterationFound = true;
        }
        if (diff > g_j && i_j != i)
          g_j = diff;
      }

      // no assignment found
      if (!assignmentInThisIterationFound)
      {
        lockedCols[j] = true;
        continue;
      }
      assignmentInThisIterationFound = false;

      //if b_j >= L + E, case 1:
      if (b_j >= (lambda + epsilon))
      {
        const Scalar diff = g_j - epsilon; // G_j - E

        const Scalar max = lambda > diff ? lambda : diff; //  max { L, G_j - E}

        //  p_j = max { L, G_j - E}
        prices[j] = max;

        //  P_i_j = a_i_jj - max {L, G_j - E}
        profits[i_j] = a_ij_j - max;

        lockedRows[i_j] = true;
        lockedCols[j] = true;

        bool newEdge = true;

        // if j_i was assigned to different i' to begin, remove (i', j_i) from S
        foreach_ (typename Edge<Scalar>::Ptr & e, *E) {
          if (e->x == i_j) // change edge
          {
            lockedCols[e->y] = false; // unlock row i'
            newEdge = false;
            e->y = j;
            e->v = a_ij_j;
            break;

          }
        }

        if (newEdge)
        {
          typename Edge<Scalar>::Ptr e = make_shared<Edge<Scalar> >();
          e->x = i_j;
          e->y = j;
          e->v = a_ij_j;
          E->push_back(e);
        }
        assignmentInThisIterationFound = true;
      }
      else  // if B_j < L + E, case 2
      {
        //  p_j = B_j - E
        prices[j] = b_j - epsilon;
        /** standard lambda scaling **/
        size_t lowerThanLambda = 0;
        Scalar newLambda = lambda;

        // if the number of objectes k with p_k < lambda is bigger than (rows - cols)
        for (size_t k = 0; k < cols; k++)
        {
          if (prices[k] < lambda) // p_k < lambda
          {
            lowerThanLambda++;
            if (prices[k] < newLambda)
              newLambda = prices[k];
          }
        }
        // set new lambda
        if (lowerThanLambda >= (cols - rows))
          lambda = newLambda;
        assignmentInThisIterationFound = false;
      }
      if (assignmentInThisIterationFound)
        assignmentFound = true;
    }
    return assignmentFound;
  }

  /**
   * returns true if p_j <= lambda for all unassigned objects.
   *
   * @param c locked columns
   * @param prices prices of objects
   * @param lambda bidding threshold
   * @return true if all prices of unassigned objects are below lambda, otherwise false
   */
  static bool unassignedObjectsLTlambda(
      const Locks & c,
      const Scalars & prices, const Scalar lambda)
  {
    for (size_t j = 0; j < c.size(); ++j)
      if (!c[j] && prices[j] > lambda)
        return false;

    return true;
  }

  /**
   * check if all persons are assigned
   * @return true if all persons are assigned, otherwise false
   */
  static bool allPersonsAssigned(const Locks & r)
  {
    for (size_t i = 0; i < r.size(); ++i)
      if (!r[i])
        return false;
    return true;
  }
};


#include <queue>

template<typename Scalar = double>
class Miller {
public:

  typedef Eigen::Matrix<Scalar, -1, -1> WeightMatrix;
  typedef Eigen::Matrix<size_t, -1, -1> AssignmentMatrix;

  /**
   * a partition represents an assignment matrix with it's
   * weight matrix
   * see Murty's algorithm for details
   */
  class Partition
  {
  public:
    Partition() : value(0)
    {
      w = WeightMatrix::Zero(w.rows(), w.cols());
    }

    Partition(
        const typename Edges<Scalar>::Ptr & edges,
        const WeightMatrix & w, const Scalar v) :
          edges(edges), w(w), value(v)
    {

    }

    typename Edges<Scalar>::Ptr edges;
    WeightMatrix w;
    Scalar value;
  };

  struct ComparePartition: std::binary_function<Partition,Partition,bool>
  {
    bool operator()(const Partition & lhs, const Partition & rhs) const
    {
      return ( lhs.value < rhs.value );
    }
  };

  /**
   * list of partitions
   */
  typedef typename std::vector<Partition> Partitions;

  /**
   * sum up values of edges, i.e. objective function value
   * @param edges
   * @return
   */
  static Scalar objectiveFunctionValue(
      const typename Edges<Scalar>::Ptr & edges)
  {
    Scalar v = 0;
    foreach_(const typename Edge<Scalar>::Ptr & e, *edges)
    {
      v += e->v;
    }
    return v;
  }

  static double CalcProb(
      const WeightMatrix & prob_mat,
      typename Edges<Scalar>::Ptr &edges)
  {
    double prob = 1.0;

    foreach_(const typename Edge<Scalar>::Ptr & e, *edges) {
      prob *= prob_mat(e->x, e->y);
    }

    return prob;
  }

  static typename std::vector<typename Edges<Scalar>::Ptr > Solve(
      const WeightMatrix & prob_mat,
      const double ratio = -1.0)
  {
    const size_t rows = prob_mat.rows(), cols = prob_mat.cols();
    assert( rows != 0 && cols != 0 && cols == rows );

    Miller<double>::WeightMatrix weight_prob;
    weight_prob.resize(rows, cols);

    double min_prob = 1.0e-6;
    double min_weight = log(min_prob);

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        double prob = prob_mat(i, j);

        if (prob > 1.0e-6) {
          weight_prob(i, j) = log(prob) - min_weight + 1.0;
        }
        else {
          weight_prob(i, j) = log(1.0e-6) - min_weight + 1.0;
        }
      }
    }

    WeightMatrix m = weight_prob;

    std::vector<typename Edges<Scalar>::Ptr > resultingEdges;

    // special case if rows = cols = 1
    if ( cols == 1 && rows == 1 )
    {
      if (m(0, 0) == 0 ) return resultingEdges;

      typename Edges<Scalar>::Ptr edges = make_shared<Edges<Scalar> >();
      edges->push_back(make_shared<Edge<Scalar> >(0, 0, m(0, 0)));
      resultingEdges.push_back(edges);
      return resultingEdges;
    }

    typename Edges<Scalar>::Ptr edges = Auction<Scalar>::solve(m); // make initial (best) assignment

    // sort edges by row
    std::sort(edges->begin(), edges->end(), Particle::PtrLesser<Edge<Scalar> >());

    // initial partition, i.e. best solution
    Partition init(edges, m, objectiveFunctionValue(edges));

    typedef std::priority_queue<Partition, std::vector<Partition>, ComparePartition > PartitionsPriorityQueue;

    // create answer-list with initial partition
    PartitionsPriorityQueue priorityQueue, answerList;
    priorityQueue.push(init);

    double threshold = CalcProb(prob_mat, init.edges) * ratio;

    // assume values between 0 and 1 !
    const Scalar lockingValue = 0.;

    while ( !priorityQueue.empty() )
    {
      // take first element from queue
      Partition currentPartition = priorityQueue.top();
      priorityQueue.pop();

      answerList.push(currentPartition);

      if (ratio < 0.0 || CalcProb(prob_mat, currentPartition.edges) <= threshold) {
        break;
      }

      // for all triplets in this solution
      for (size_t e = 0; e < currentPartition.edges->size(); ++e)
      {
        typename Edge<Scalar>::Ptr &triplet = currentPartition.edges->at(e);

        WeightMatrix P_ = currentPartition.w; // P' = P

        // exclude edge by setting weight in matrix to lockingValue -> NOT (x, y)
        P_(triplet->x, triplet->y) = lockingValue;

        // determine solution for changed matrix and create partition
        typename Edges<Scalar>::Ptr S_ = Auction<Scalar>::solve(P_);

        if (S_->size() == P_.rows())// solution found?
        {
          // sort edges by row
          std::sort(
              S_->begin(), S_->end(),
              Particle::PtrLesser<Edge<Scalar> >());

          Partition newPartition(S_, P_, objectiveFunctionValue(S_));

          // if S exists
          priorityQueue.push(newPartition);// push back unpartitioned new partition
        }
        // remove all vertices that include row and column of current node
        // i.e. force using this edge
        for (size_t r = 0; r < currentPartition.w.rows(); ++r )
          currentPartition.w(r, triplet->y) = lockingValue;

        for (size_t c = 0; c < currentPartition.w.cols(); ++c )
          currentPartition.w(triplet->x, c) = lockingValue;

        // set edge back to original value
        currentPartition.w(triplet->x, triplet->y) = triplet->v =
            m(triplet->x, triplet->y);
      }
    }

    // create return list
    while( !answerList.empty() )
    {
      resultingEdges.push_back(answerList.top().edges);
      answerList.pop();
    }

    return resultingEdges;
  }

};


#endif /* MURTY_H_ */
