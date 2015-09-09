/*
 * gaussian.h
 *
 *  Created on: Feb 9, 2014
 *      Author: baj
 */

#ifndef GAUSSIAN_H_
#define GAUSSIAN_H_

#include <vector>
#include "statistic.h"
#include "xmlUtil.h"

class Gaussian
{
  Gaussian();
  virtual ~Gaussian();

public:
  static Gaussian &ins();

  double cdf(const double mu, const double segma, double x);
  double pdf(const double mu, const double segma, double x);

private:
  double mCDF[5000];
  bool mLoaded;
};


// A simple random number generator based on George Marsaglia's MWC (Multiply With Carry) generator.
// This is not intended to take the place of the library's primary generator, Mersenne Twister.
// Its primary benefit is that it is simple to extract its state.

class SimpleRNG
{
private:
  SimpleRNG();

public:
  static SimpleRNG &ins();

  int operator()(int N) {
    return GetRand() % N;
  }

  int GetRand();

  // A uniform random sample from the open interval (0, 1)
  double GetUniform(double low = 0.0, double high = 1.0);

  // Normal (Gaussian) random sample
  double GetNormal(double mean = 0.0, double standardDeviation = 1.0);

  // Exponential random sample
  double GetExponential(double mean = 1.0);

  // Gamma random sample
  double GetGamma(double shape, double scale);

  // Chi-square sample
  double GetChiSquare(double degreesOfFreedom);

  // Inverse-gamma sample
  double GetInverseGamma(double shape, double scale);

  // Weibull sample
  double GetWeibull(double shape, double scale);

  // Cauchy sample
  double GetCauchy(double median, double scale);

  // Student-t sample
  double GetStudentT(double degreesOfFreedom);

  // The Laplace distribution is also known as the double exponential distribution.
  double GetLaplace(double mean, double scale);

  // Log-normal sample
  double GetLogNormal(double mu, double sigma);

  // Beta sample
  double GetBeta(double a, double b);

  // Poisson sample
  int GetPoisson(double lambda);

  void RandomSeeding(int seed);

private:
  int PoissonLarge(double lambda);
  int PoissonSmall(double lambda);
  double LogFactorial(int n);

  pthread_t hash(const pthread_t &i) {
    return i * 2654435761;
  }

private:
  enum {
    HashSize = 1024
  };

  drand48_data mStatus_drand48[HashSize];
  uint mStatus_rand[HashSize];

  static const double PI;
};


#endif /* GAUSSIAN_H_ */
