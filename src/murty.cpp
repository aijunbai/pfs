/*
 * murty.cpp
 *
 *  Created on: Jan 20, 2014
 *      Author: baj
 */

#include <cmath>
#include <iostream>
#include <sstream>
#include "tracker.h"
#include "terminal_utils.h"
#include <murty.h>

using namespace std;

namespace {
template <class T, class TAl>
inline T* begin_ptr(std::vector<T,TAl>& v)
{return  v.empty() ? NULL : &v[0];}

template <class T, class TAl>
inline const T* begin_ptr(const std::vector<T,TAl>& v)
{return  v.empty() ? NULL : &v[0];}
}

STATISTIC::Ptr Murty::mTimeUsageStat;
STATISTIC::Ptr Murty::mMurtySizeStat;

Murty::Murty()
{
  if (Params::ins().threads == 0) {
    PRINT_ERROR("Murty::Murty(): Params::ins().threads == 0");
    abort();
  }
}

Murty::~Murty()
{
  if (Params::ins().threads == 0) {
    PRINT_ERROR("Murty::~Murty(): Params::ins().threads == 0");
    abort();
  }
}

Murty &Murty::ins()
{
  static Murty murty;

  return murty;
}

Particle::Permutations::Ptr Murty::Solve(
    HashMap<int, HashMap<int, double> > &prob_mat,
    Combination::Table<int> &obs_set,
    Combination::Table<int> &hs_set)
{
  std::vector<double> buffer;

  assert(obs_set.size() == hs_set.size());

  uint size = obs_set.size();
  int len = size * size + 1;
  buffer.resize(len);

  for (int i = 0; i < len - 1; ++i) {
    int row = i / size;
    int col = i % size;

    buffer[i] = prob_mat[obs_set(row)][hs_set(col)];
  }
  buffer[len - 1] = Params::ins().assignments_pruning;

  return SolveImp(buffer);
}

Particle::Permutation::Ptr Murty::Solve(
    std::vector<Simulator::Human::Ptr> &objects,
    std::vector<IdentifiedHuman::Ptr> &hypothese)
{
  std::vector<double> buffer;

  assert(objects.size() == hypothese.size());

  uint size = objects.size();
  Particle::Permutation::Ptr perm =
      make_shared<Particle::Permutation>(0.0, 1.0);

  if (size <= 0) {
    return perm;
  }

  if (size <= 1) {
    perm->push_back(0);

    return perm;
  }

  int len = size * size + 1;
  buffer.resize(len);
  fill(buffer.begin(), buffer.end(), 1.0e-6);

  for (uint i = 0; i < size; ++i) {
    for (uint j = 0; j < size; ++j) {
      double len = FLT_MAX;

      if (objects[i] && hypothese[j]) {
        len =
            (objects[i]->Position() -
                hypothese[j]->ExpectedState()->Position()).length();
      }

      double prob = exp(-len);

      int k = i * size + j;
      buffer[k] = prob;
    }
  }
  buffer[len - 1] = -1.0;

  Particle::Permutations::Ptr perms = SolveImp(buffer);
  if (!perms->empty()) {
    perm = perms->front();
  }

  return perm;
}

Particle::Permutation::Ptr Murty::Solve(
    HashMap<int, HashMap<int, double> > &oi_prob,
    uint rows,
    std::vector<IdentifiedHuman::Ptr> &identified_humans
)
{
  std::vector<double> buffer;

  Particle::Permutation::Ptr perm =
      make_shared<Particle::Permutation>(0.0, 1.0);

  uint cols = identified_humans.size();

  assert(rows > 0);
  assert(rows <= cols);

  if (cols <= 0) {
    return perm;
  }

  if (cols <= 1) {
    perm->push_back(0);

    return perm;
  }

  if (rows == 1) {
    double max = -1.0;
    int ih = -1;

    for (uint i = 0; i < cols; ++i) {
      double prob = oi_prob[0][identified_humans[i]->mID];

      if (prob > max) {
        max = prob;
        ih = i;
      }
    }

    if (ih >= 0) {
      perm->push_back(ih);
    }

    return perm;
  }

  int len = cols * cols + 1;
  buffer.resize(len);
  fill(buffer.begin(), buffer.end(), 1.0e-6);

  for (uint o = 0; o < rows; ++o) {
    for (uint ih = 0; ih < cols; ++ih) {
      double prob = oi_prob[o][identified_humans[ih]->mID];

      if (prob > 1.0e-6) {
        int k = o * cols + ih;
        buffer[k] = prob;
      }
    }
  }
  buffer[len - 1] = -1.0;

  Particle::Permutations::Ptr perms = SolveImp(buffer);
  if (!perms->empty()) {
    perm = perms->front();
  }

  return perm;
}

Particle::Permutation::Ptr Murty::Solve(
    HashMap<int, HashMap<int, double> > &prob_mat,
    uint cols, Particle &particle)
{
  std::vector<double> buffer;

  Particle::Permutation::Ptr perm =
      make_shared<Particle::Permutation>(0.0, 1.0);

  uint rows = particle.mHumans.size();

  assert(rows > 0);
  assert(rows <= cols);

  if (cols <= 0) {
    return perm;
  }

  if (cols <= 1) {
    perm->push_back(0);

    return perm;
  }

  if (rows == 1) {
    HumanState *hs = particle.mHumans.get(0);
    double max = -1.0;
    int ih = -1;

    for (uint i = 0; i < cols; ++i) {
      double prob = prob_mat[hs->mID][i];

      if (prob > max) {
        max = prob;
        ih = i;
      }
    }

    if (ih >= 0) {
      perm->push_back(ih);
    }

    return perm;
  }

  int len = cols * cols + 1;
  buffer.resize(len);
  fill(buffer.begin(), buffer.end(), 1.0e-6);

  for (uint i = 0; i < rows; ++i) {
    HumanState *hs = particle.mHumans.get(i);

    for (uint j = 0; j < cols; ++j) {
      double prob = prob_mat[hs->mID][j];

      if (prob > 1.0e-6) {
        int k = i * cols + j;
        buffer[k] = prob;
      }
    }
  }
  buffer[len - 1] = -1.0;

  Particle::Permutations::Ptr perms = SolveImp(buffer);
  if (!perms->empty()) {
    perm = perms->front();
  }

  return perm;
}

Particle::Permutations::Ptr Murty::CallMiller(vector<double> &buf)
{
  Particle::Permutations::Ptr perms =
      make_shared<Particle::Permutations>(0.0);

  uint len = buf.size();
  uint size = rint(sqrt(len - 1));
  assert(size * size + 1 == len);

  double ratio = buf[len - 1];
  int n = (int) rint( sqrt(len - 1) );

  Miller<double>::WeightMatrix prob_mat;
  prob_mat.resize(n, n);

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      prob_mat(i, j) = buf[i * n + j];
    }
  }

  STATISTIC::Add(mMurtySizeStat, size);

  std::vector<Edges<>::Ptr> ret =
      Miller<>::Solve(prob_mat, ratio);

  foreach_(Edges<>::Ptr &edges, ret) {
    Particle::Permutation::Ptr perm =
        make_shared<Particle::Permutation>(0.0, 1.0);

    perm->resize(edges->size(), -1);
    double prob = 1.0;

    foreach_(Edge<>::Ptr &e, *edges) {
      int row = e->x;
      int col = e->y;

      prob *= prob_mat(row, col);

      if (!has(*perm, col)) {
        perm->at(row) = col;
      }
      else {
        TerminalLogger::log() << "Miller bug" << endl;

        perm->clear();
        break;
      }
    }

    perm->prob1 = prob;

    if (perm->size()) {
      perms->total_prob += perm->prob();
      perms->push_back(perm);
    }
  }

  return perms;
}

Particle::Permutations::Ptr Murty::SolveImp(vector<double> &buf)
{
  return CallMiller(buf);
}

void Murty::PrintStatistics()
{
  STATISTIC::Print(mTimeUsageStat,
                     "murty algorithm computation time (ms)");
  STATISTIC::Print(mMurtySizeStat,
                     "murty algorithm called size (ms)");
}

void Murty::CreateStatistics()
{
  STATISTIC::Create(mTimeUsageStat);
  STATISTIC::Create(mMurtySizeStat);
}
