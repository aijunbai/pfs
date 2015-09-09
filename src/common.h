/*
 * common.h
 *
 *  Created on: Jan 14, 2014
 *      Author: baj
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <functional>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/make_shared.hpp>

#include "geometry.h"
#include "util.h"
#include "params.h"
#include "gaussian.h"

#include <set>
#include <map>
#include <tr1/unordered_map>
#include <tr1/unordered_set>

#define DEFAULT_LOG_NAME "human_tracker"
#define DEFAULT_LOG_PATH "../.logs/"

#define foreach_         BOOST_FOREACH
#define foreach_r_       BOOST_REVERSE_FOREACH

using boost::shared_ptr;
using boost::shared_array;
using boost::make_shared;

#define FILL_ARRAY(type, a, x) \
    do { \
      foreach_(type &_r, (a)) { \
        _r = (x); \
      } \
    } while (0)

#ifdef NDEBUG
template<typename K, typename V>
struct HashMap : std::tr1::unordered_map<K, V> {};

template<typename K>
struct HashSet : std::tr1::unordered_set<K> {};
#else
template<typename K, typename V>
struct HashMap : std::map<K, V> {};

template<typename K>
struct HashSet : std::set<K> {};
#endif

namespace std {
namespace tr1 {

template<class T>
struct hash<boost::shared_ptr<T> > {
    size_t operator()(const boost::shared_ptr<T>& key) const {
        return (size_t) key.get();
    }
};

}
}

template<class _Container, typename _Tp>
bool has(const _Container &x, const _Tp &key) {
  return std::find(x.begin(), x.end(), key) != x.end();
}

struct RobotPose {
  vector2d mPosition;
  double mAngle;

  RobotPose(): mPosition(0.0, 0.0), mAngle(0.0) {

  }
};

template<typename _Tp>
inline const _Tp&
Max(const _Tp& x, const _Tp& y)
{
  return std::max(x, y);
}

template<typename _Tp>
inline const _Tp&
Min(const _Tp& x, const _Tp& y)
{
  return std::min(x, y);
}

template<typename _Tp>
inline const _Tp&
MinMax(const _Tp& min, const _Tp& x, const _Tp& max)
{
  return Min(Max(min, x), max);
}

template<typename _Tp>
inline int
Sign(const _Tp& x)
{
  return x >= 0? 1: -1;
}

namespace _angle {

inline double GetNormalizeAngleDeg(
    double ang, const double min_ang = -180.0)
{
  if (ang < min_ang) {
    do {
      ang += 360.0;
    } while (ang < min_ang);
  }
  else {
    const double max_ang = 360.0 + min_ang;

    while (ang >= max_ang){
      ang -= 360.0;
    }
  }

  return ang;
}

inline double GetNormalizeAngleRad(
    double ang, const double min_ang = -M_PI)
{
  while (ang > 2.0 * M_PI + min_ang)
  {
    ang -= 2.0 * M_PI;
  }

  while (ang < min_ang)
  {
    ang += 2.0 * M_PI;
  }

  return ang;
}

inline bool IsAngleDegInBetween(
    const double alpha, const double gamma, const double beta)
{
  return GetNormalizeAngleDeg(gamma, alpha) <=
      GetNormalizeAngleDeg(beta, alpha);
}

inline bool IsAngleRadInBetween(
    const double alpha, const double gamma, const double beta)
{
  return GetNormalizeAngleRad(gamma, alpha) <=
      GetNormalizeAngleDeg(beta, alpha);
}

}

namespace _parser {

inline double get_double(char **str_ptr){
  while (!isdigit(**str_ptr) && **str_ptr != '-'
      && **str_ptr != '+' && **str_ptr != '.' && **str_ptr) (*str_ptr)++;
  return strtod(*str_ptr, str_ptr);
}

inline int get_int(char **str_ptr){
  while (!isdigit(**str_ptr) && **str_ptr != '-'
      && **str_ptr != '+' && **str_ptr != '.' && **str_ptr) (*str_ptr)++;
  return static_cast<int>(strtol(*str_ptr, str_ptr, 10));
}

}

namespace _motion {

inline vector2d RandomPosition(
    double left, double right, double top, double bottom)
{
  double x = SimpleRNG::ins().GetUniform(left, right);
  double y = SimpleRNG::ins().GetUniform(top, bottom);

  return vector2d(x, y);
}

inline double RandomAngle()
{
  return SimpleRNG::ins().GetUniform(0.0, M_2PI);
}

inline double RandomSpeed()
{
  return SimpleRNG::ins().GetUniform(
      -Params::ins().max_speed, Params::ins().max_speed);
}

inline vector2d RandomVelocity()
{
  return vector2d(RandomSpeed(), 0.0).rotate(RandomAngle());
}

inline double RandomPower(double mean = 0.0, double segama = 1.0)
{
  return SimpleRNG::ins().GetNormal(mean, segama);
}

inline vector2d NoiseVector(double error = 0.05)
{
  return vector2d(
      SimpleRNG::ins().GetNormal(0.0, error),
      SimpleRNG::ins().GetNormal(0.0, error));
}

inline double NoiseAngle(double error = 5.0)
{
  return SimpleRNG::ins().GetNormal(0.0, RAD(error));
}

}


#endif /* COMMON_H_ */
