/*
 * combination.h
 *
 *  Created on: Jan 14, 2014
 *      Author: baj
 */

#ifndef COMBINATION_H_
#define COMBINATION_H_

#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>

#include "common.h"
#include "logger.h"
#include "xmlUtil.h"

class Combination
{
private:
  Combination(std::string cache);
  virtual ~Combination();

public:
  static Combination &ins();

public:
  typedef long long unsigned int big_int_t;

  template<class DataType>
  class Table: public HashMap<uint, DataType> {
    friend class Combination;

  public:
    DataType &operator()(const uint &key) {
      assert(key < this->size());
      assert(this->count(key));

      return this->operator [](key);
    }
  };

private:
  void GenerateCombinationTable(int begin, int end);
  void GenerateSubSets(std::vector<int> &set, int m);

  void EnumerateSubSets(
      std::vector<int> &set,
      int m, int k,
      shared_ptr<std::vector<int> > &l);

  template<class T>
  void Dump(std::ostream &os, T &o, std::vector<int> &keys) {
    for (typename T::const_iterator it = o.begin(); it != o.end(); ++it) {
      keys.push_back(it->first);
      Dump(os, it->second, keys);
      keys.pop_back();
    }
  }

  void Save(const char *file_name);
  bool Load(const char *file_name);

public:
  static big_int_t Factorial(uint N);
  static big_int_t Binomial(uint N, uint K);

  static const uint FACT_SIZE = 21;

public:
  uint Size() const {
    return mCombination.size();
  }

  Table<Table<Table<int> > > &operator()(uint n) {
    if (n >= Size()) {
      GenerateCombinationTable(Size(), n + 1);
    }

    return mCombination(n);
  }

private:
  Table<Table<Table<Table<int> > > > mCombination;
  std::string mCache;
  bool mLoaded;
};

template<>
void Combination::Dump(std::ostream &os, const int &o, std::vector<int> &keys);

#endif /* COMBINATION_H_ */
