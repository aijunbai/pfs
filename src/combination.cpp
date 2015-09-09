/*
 * combination.cpp
 *
 *  Created on: Jan 14, 2014
 *      Author: baj
 */

#include <cmath>
#include <combination.h>
#include "logger.h"

Combination::Combination(std::string cache): mCache(cache), mLoaded(false)
{
  mLoaded = Load(mCache.c_str());

  if (!mLoaded) {
    GenerateCombinationTable(0, 16);
  }
}

Combination::~Combination()
{
  if (!mLoaded) {
    Save(mCache.c_str());
  }
}

Combination &Combination::ins()
{
  static Combination combination("./config/combination.tbl");

  return combination;
}

void Combination::GenerateCombinationTable(int begin, int end)
{
  SCOPED_LOCK_

  for (int n = begin; n < end; ++n) {
    std::vector<int> set;

    for (int i = 0; i < n; ++i) {
      set.push_back(i);
    }

    for (int m = 0; m <= n; ++m) {
      GenerateSubSets(set, m);

      assert(mCombination(n)(m).size() == Binomial(n, m));
      TerminalLogger::log() << m << " things of " << n << ": " << mCombination(n)(m).size() << std::endl;
    }
  }
  TerminalLogger::log() << std::endl;
}

void Combination::GenerateSubSets(std::vector<int> &set, int m)
{
  shared_ptr<std::vector<int> > l = make_shared<std::vector<int> >();
  EnumerateSubSets(set, m, 0, l);
}

template<>
void Combination::Dump(std::ostream &os, const int &o, std::vector<int> &keys) {
  for (uint i = 0; i < keys.size(); ++i) {
    os << keys[i] << " ";
  }

  os << o << "\n";
}

Combination::big_int_t Combination::Factorial(uint N)
{
  static big_int_t fact[] = {
      /*0*/  1,
      /*1*/  1,
      /*2*/  2,
      /*3*/  6,
      /*4*/  24,
      /*5*/  120,
      /*6*/  720,
      /*7*/  5040,
      /*8*/  40320,
      /*9*/  362880,
      /*10*/  3628800,
      /*11*/  39916800,
      /*12*/  479001600,
      /*13*/  6227020800,
      /*14*/  87178291200,
      /*15*/  1307674368000,
      /*16*/  20922789888000,
      /*17*/  355687428096000,
      /*18*/  6402373705728000,
      /*19*/  121645100408832000,
      /*20*/  2432902008176640000
  };

  if (N < FACT_SIZE) {
    return fact[N];
  }

  return floor(sqrt(2.0 * M_PI * N) * pow(N / M_E, N)); //Stirling's approximation
}

Combination::big_int_t Combination::Binomial(uint N, uint K)
{
  if (N < FACT_SIZE && N - K < FACT_SIZE) {
    big_int_t c = Factorial(N) / Factorial(K) / Factorial(N - K);

    return c;
  }

  if( K > N - K )
    K = N - K;

  big_int_t c = 1;

  for(uint i = 0; i < K; ++i) {
    c *= (N - i);
    c /= (i + 1);
  }

  return c;
}

void Combination::Save(const char *file_name)
{
  std::ofstream fout(file_name);

  if (fout.good()) {
    std::vector<int> keys;

    Dump(fout, mCombination, keys);
  }

  fout.close();
}

bool Combination::Load(const char *file_name)
{
  std::ifstream fin(file_name);

  int n, m, c, i, k;
  if (fin.good()) {
    while (fin >> n >> m >> c >> i >> k) {
      mCombination[n][m][c][i] = k;
    }
    fin.close();

    TerminalLogger::log() << "Loaded combination table with size " << Size() << std::endl;

    return true;
  }

  fin.close();

  return false;
}

void Combination::EnumerateSubSets(
    std::vector<int> &set,
    int m, int k,
    shared_ptr<std::vector<int> > &l)
{
  if (m == 0) {
    int n = set.size();
    int c = mCombination[n][l->size()].size();

    if (l->empty()) {
      mCombination[n][0][c][0] = -1;
    }
    else {
      for (uint i = 0; i < l->size(); ++i) {
        mCombination[n][l->size()][c][i] = l->at(i);
      }
    }
  }
  else {
    for(uint i = k; i < set.size(); ++i) {
      l->push_back(set[i]);
      EnumerateSubSets(set, m-1, i+1, l);
      l->pop_back();
    }
  }
}
