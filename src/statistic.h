#ifndef STATISTIC_H
#define STATISTIC_H

#include <math.h>
#include "logger.h"
#include "common.h"
#include "xmlUtil.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using boost::shared_ptr;
using boost::make_shared;

typedef long long int count_t;

class STATISTIC
{
public:
  typedef shared_ptr<STATISTIC> Ptr;

public:
  STATISTIC();
  STATISTIC(double val, count_t count);

  void Add(double val);
  void Print(const std::string& name) const;
  void Clear();
  count_t GetCount() const;
  void Initialise(double val, count_t count);
  double GetTotal() const;
  double GetMean() const;
  double GetVariance() const;
  double GetStdDev() const;
  double GetStdErr() const;
  double GetMax() const;
  double GetMin() const;

  static void Create(Ptr &p)
  {
    if (!p) {
      p = make_shared<STATISTIC>();
    }
  }

  static void Add(Ptr &p, double val)
  {
    if (p) {
      p->Add(val);
    }
  }

  static void Print(Ptr &p, const std::string& name)
  {
    if (p) {
      p->Print(name);
    }
  }

private:
  count_t Count;
  double Mean;
  double Variance;
  double Min, Max;
};

inline STATISTIC::STATISTIC()
{
  Clear();
}

inline STATISTIC::STATISTIC(double val, count_t count)
{
  Initialise(val, count);
}

inline void STATISTIC::Add(double val)
{
  double meanOld = Mean;
  count_t countOld = Count;
  ++Count;

  Mean += (val - Mean) / Count;
  Variance = (countOld * (Variance + meanOld * meanOld)
      + val * val) / Count - Mean * Mean;
  if (val > Max)
    Max = val;
  if (val < Min)
    Min = val;
}

inline void STATISTIC::Clear()
{
  Count = 0;
  Mean = 0;
  Variance = 0;
  Min = +1.0e6;
  Max = -1.0e6;
}

inline count_t STATISTIC::GetCount() const
{
  return Count;
}

inline void STATISTIC::Initialise(double val, count_t count)
{
  Count = count;
  Mean = val;
}

inline double STATISTIC::GetTotal() const
{
  return Mean * Count;
}

inline double STATISTIC::GetMean() const
{
  return Mean;
}

inline double STATISTIC::GetStdDev() const
{
  return sqrt(Variance);
}

inline double STATISTIC::GetStdErr() const
{
  return sqrt(Variance / Count);
}

inline double STATISTIC::GetMax() const
{
  return Max;
}

inline double STATISTIC::GetMin() const
{
  return Min;
}

inline void STATISTIC::Print(const std::string& name) const
{
  TerminalLogger::log() << "#avg "
      << name
      << ": " << Mean
      << " (" << GetCount()
          << ") [" << Min
                 << ", " << Max
                 << "] +- error=" << GetStdErr()
                 << ", sigma=" << GetStdDev()
                 << std::endl;
}


#endif // STATISTIC
