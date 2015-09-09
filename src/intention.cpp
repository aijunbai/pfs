/*
 * intention.cpp
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#include <intention.h>
#include <cstdlib>
#include <cassert>

using namespace std;

uint HumanIntention::mIDCounter = 0;

IntentionDistribution::Initializer IntentionDistribution::mInitializers[3] =
  {
   &IntentionDistribution::RandomVertex,
   &IntentionDistribution::RandomSimplex,
   &IntentionDistribution::RandomUniform
  };

HumanIntention::HumanIntention(std::string name, HumanTracker *tracker):
    mID(mIDCounter++), mName(name), mTracker(tracker)
{

}

HumanIntention::~HumanIntention()
{
}

IntentionFactory::IntentionFactory()
{
}

IntentionFactory::~IntentionFactory()
{

}

IntentionFactory &IntentionFactory::ins()
{
  static IntentionFactory factory;

  return factory;
}

void IntentionFactory::Register(shared_ptr<HumanIntention> intention)
{
  assert (mIntentionMap[intention->GetName()] == 0);

  mIntentionMap[intention->GetName()] = intention;
  mIntentions.push_back(intention);
}

HumanIntention *IntentionFactory::GetIntention(const std::string &name)
{
  return mIntentionMap[name].get();
}

HumanIntention *IntentionFactory::GetIntention(uint iid) const
{
  assert(iid < mIntentions.size());
  assert(mIntentions[iid]->mID == iid);

  if (iid < mIntentions.size()) {
    return mIntentions[iid].get();
  }

  return 0;
}

HumanIntention *IntentionFactory::GetRandomIntention() const
{
  return GetIntention(SimpleRNG::ins().GetRand() % mIntentions.size());
}

const vector<shared_ptr<HumanIntention> > &IntentionFactory::GetIntentions() const
{
  return mIntentions;
}

Eigen::MatrixXd IntentionFactory::GetTransitionMatrix(const HumanState &hs, double duration)
{
  int size = IntentionFactory::ins().Size();
  Eigen::MatrixXd trans_matrix(Eigen::MatrixXd::Zero(size, size));

  foreach_(shared_ptr<HumanIntention> &i, mIntentions) {
    HashMap<HumanIntention*, double> trans =
        i->GetTransitions(hs, duration);

    foreach_(shared_ptr<HumanIntention> &j, mIntentions) {
      trans_matrix(j->mID, i->mID) = trans[j.get()];
    }
  }

  return trans_matrix;
}
