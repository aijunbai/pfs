/*
 * simulator.cpp
 *
 *  Created on: Feb 3, 2014
 *      Author: baj
 */

#include <boost/math/distributions/negative_binomial.hpp>
#include <boost/math/distributions/normal.hpp>
#include <sstream>

#include "timer.h"
#include <simulator.h>
#include "tracker.h"
#include "task.h"
#include "params.h"
#include "murty.h"

using namespace std;

Simulator::Simulator(Task *task, HumanTracker *tracker):
        mTask(task),
        mTracker(tracker)
{
  if (Params::ins().CLEAR_distance_threshold > 0.0) {
    double confidence = 0.0;

    do {
      if (confidence > 1.0 + FLT_EPSILON) {
        break;
      }

      mMetrics.insert(make_shared<EvaluationMetrics>(
              mTracker, this,
              Params::ins().CLEAR_distance_threshold, false, confidence));

      confidence += 0.05;
    } while (1);

    mMetrics.insert(make_shared<EvaluationMetrics>(
            mTracker, this,
            Params::ins().CLEAR_distance_threshold, true));
  }
}

Simulator::~Simulator()
{
  EvaluationMetrics::Ptr best;
  foreach_(const EvaluationMetrics::Ptr &i, mMetrics) {
    i->Complete();

    if (!best || i->MOTA() > best->MOTA()) {
      best = i;
    }
  }

  mMetrics.clear();
  PRINT_VALUE(best);
}

Simulator::EvaluationMetrics::~EvaluationMetrics()
{
  PrintCLEARValues();
}

void Simulator::EvaluationMetrics::Complete()
{
  for (HashMap<uint, Tracks>::iterator it = mTracks.begin();
      it != mTracks.end(); ++it) {
    double length = it->second.size();
    bool tracked = false;
    double n_tracked = 0.0;
    double n_fragmetation = 0.0;

    foreach_(Track &track, it->second) {
      if (track.second) {
        tracked = true;
        n_tracked += 1.0;
      }

      if (tracked && !track.second) {
        tracked = false;
        n_fragmetation += 1.0;
      }
    }

    double ratio = n_tracked / length;

    if (ratio >= 0.8) {
      mMT += 1.0;
    }
    else if (ratio >= 0.2) {
      mPT += 1.0;
    }
    else {
      mML += 1.0;
    }

    mFM += n_fragmetation;
  }
}

void Simulator::EvaluationMetrics::PrintCLEARValues()
{
  TerminalLogger::log() << endl;

  if (weighted) {
    PRINT_VALUE(weighted);
  }
  else {
    PRINT_VALUE(confidence_threshold);
  }

  if (mG) {
    PRINT_VALUE(mC);
    PRINT_VALUE(mD);
    PRINT_VALUE(mI);
    PRINT_VALUE(mM);
    PRINT_VALUE(mFP);
    PRINT_VALUE(mMME);
    PRINT_VALUE(mM+mFP+mMME);
    PRINT_VALUE(mG);

    PRINT_VALUE(mMT);
    PRINT_VALUE(mPT);
    PRINT_VALUE(mML);
    PRINT_VALUE(mFM);

    PRINT_VALUE(Precision());
    PRINT_VALUE(Recall());

    PRINT_VALUE(MOTP());
    PRINT_VALUE(MHIP());

    if (weighted) {
      TerminalLogger::log() << "MOTA() = '" << MOTA() <<
          "' W = '" << weighted << "'" << endl;
    }
    else {
      TerminalLogger::log() << "MOTA() = '" << MOTA() <<
          "' CT = '" << confidence_threshold << "'" << endl;
    }

    TerminalLogger::log() << endl;
  }
}

void Simulator::Run(int argc, char **argv)
{
  Params::ins().Parse(argc, argv, true); //parse options again, since options may be overwritten by simulator constrcution function

  Initialize(Params::ins().simulator_expected_humans);

  double death_rate = Params::ins().death_rate;
  double birth_rate = Params::ins().simulator_expected_humans * death_rate;

  while (Params::ins().runto_step < 0
      || int(mTracker->mCurrentStep) <= Params::ins().runto_step) {

    SimpleRNG::ins().RandomSeeding(mTracker->mCurrentStep); //sync random seed for debug purposes

    Observation::Ptr obs = make_shared<Observation>();
    double duration = Params::ins().sleep_duration;

    Simulate(duration, birth_rate, death_rate);
    GenerateObservations(*obs, duration);
    CheckConsistence(*obs);

    foreach_(Human::Ptr &human, mHumans) {
      human->mTrajectory.push_back(human->Position());

      vector<vector2d>::iterator it = human->mTrajectory.begin();
      while (human->mTrajectory.size() > Params::ins().trajectory_size) {
        it = human->mTrajectory.erase(it);
      }
    }

    if (mTracker->mRCGLogger) {
      foreach_(const Human::Ptr &human, mHumans) {
        human->Log(
            mTracker->mRCGLogger[HumanTracker::LOG_STATE],
            true, "Human");

        human->Log(
            mTracker->mRCGLogger[HumanTracker::LOG_STATE_OBS],
            true, "Human");

        human->Log(
            mTracker->mRCGLogger[HumanTracker::LOG_ALL],
            true, "Human");

        if (mTracker->mDebugLevel > 4) {
          for (int i = 0; i < int(human->mTrajectory.size()) - 1; ++i) {
            vector2d from = human->mTrajectory.at(i);
            vector2d to = human->mTrajectory.at(i+1);

            mTracker->mRCGLogger[HumanTracker::LOG_STATE]->LogLine(
                from.x, from.y, to.x, to.y, Qt::gray);

            mTracker->mRCGLogger[HumanTracker::LOG_STATE_OBS]->LogLine(
                from.x, from.y, to.x, to.y, Qt::gray);

            mTracker->mRCGLogger[HumanTracker::LOG_ALL]->LogLine(
                from.x, from.y, to.x, to.y, Qt::gray);
          }
        }
      }
    }

    LogRCG();
    mTracker->Update(obs, duration);

    if (Params::ins().interface) {
      Display();
    }

    foreach_(const EvaluationMetrics::Ptr &i, mMetrics) {
      i->Evaluate();
    }

    if (End()) {
//      Initialize(expected_humans);
      break;
    }
  }
}

void Simulator::GenerateObservations(Observation &obs, double duration)
{
  foreach_(Human::Ptr &hs, mHumans) {
    if (InFieldOfView(hs->Position())) {
      double x = SimpleRNG::ins().GetNormal(0.0, 0.025);
      double y = SimpleRNG::ins().GetNormal(0.0, 0.025);
      double conf = Observation::SampleTrueConfidence();

      obs.mDetections.push_back(
          make_shared<Detection>(
              hs->Position() + vector2d(x, y), -1.0, conf,
              -1.0, -1.0, -1.0, -1.0));
    }
  }

  if (!obs.mDetections.empty() && Params::ins().missing_rate > 0.0) {
    const int m = SimpleRNG::ins().GetPoisson(
        Params::ins().missing_rate * obs.mDetections.size() * duration);
    for (int i = 0; i < m; ++i) {
      obs.mDetections.erase(
          obs.mDetections.begin() + SimpleRNG::ins().GetRand() % obs.mDetections.size());
      if (obs.mDetections.empty()) break;
    }
  }

  if (Params::ins().false_rate > 0.0) {
    const int n = SimpleRNG::ins().GetPoisson(Params::ins().false_rate * duration);

    for (int i = 0; i < n; ++i) {
      vector2d pos = mTracker->FalseDetection();
      double conf = Observation::SampleFalseConfidence();

      obs.mDetections.push_back(make_shared<Detection>(
          pos, -1.0, conf, -1.0, -1.0, -1.0, -1.0));
    }
  }
}

Simulator::Human::Human(
    const vector2d &pos,
    const vector2d &target,
    HumanIntention *intention):
        HumanState(),
        mTarget(target)
{
  mPosition = pos;
  SetIntention(intention);
}

void Simulator::EvaluationMetrics::Evaluate()
{
  if (distance_threshold < 0.0) {
    return;
  }

  vector<IdentifiedHuman::Ptr> identified_humans;

  foreach_(IdentifiedHuman::Ptr &ih, mTracker->IdentifiedHumans()) {
    if ((ih->Confidence() >= confidence_threshold - FLT_EPSILON || weighted) &&
        mSimulator->InFieldOfView(ih->ExpectedState()->mPosition)) {
      identified_humans.push_back(ih);
    }
  }

  set<Mapping> clear_mapping;

  foreach_(const Mapping &mapping, mCLEARMapping) {
    if (has(identified_humans, mapping.second) &&
        has(mSimulator->mHumans, mapping.first) &&
        mSimulator->InFieldOfView(mapping.first->Position()))
    {
      double len = (mapping.first->Position()
          - mapping.second->ExpectedState()->Position()).length();

      if (len < distance_threshold + FLT_EPSILON) {
        if (weighted) {
          double conf = mapping.second->Confidence();

          mC += conf;
          mD += conf * len;
          mI += conf * (mapping.first->Intention()
              == mapping.second->mIntention);
        }
        else {
          mC += 1.0;
          mD += len;
          mI += (mapping.first->Intention()
              == mapping.second->mIntention);
        }

        clear_mapping.insert(mapping);
      }
    }
  }

  vector<Human::Ptr> objects;
  vector<IdentifiedHuman::Ptr> hypothese;

  foreach_(Human::Ptr &real, mSimulator->mHumans) {
    bool found = false;
    foreach_(const Mapping &mapping, clear_mapping) {
      if (mapping.first == real) {
        found = true;
        break;
      }
    }

    if (!found && mSimulator->InFieldOfView(real->Position())) {
      objects.push_back(real);
    }
  }

  foreach_(IdentifiedHuman::Ptr &ih, identified_humans) {
    bool found = false;
    foreach_(const Mapping &mapping, clear_mapping) {
      if (mapping.second == ih) {
        found = true;
        break;
      }
    }

    if (!found) {
      hypothese.push_back(ih);
    }
  }

  while (objects.size() > hypothese.size()) {
    hypothese.push_back(IdentifiedHuman::Ptr());
  }

  while (hypothese.size() > objects.size()) {
    objects.push_back(Human::Ptr());
  }

  set<Mapping> mmes;
  Particle::Permutation::Ptr permutation =
      Murty::ins().Solve(objects, hypothese);

  for (uint i = 0; i < objects.size(); ++i) {
    if (objects[i]) {
       if (i >= permutation->size()) {
        continue;
      }

      uint j = permutation->at(i);

      if (hypothese[j]) {
        double len =
            (objects[i]->Position() -
                hypothese[j]->ExpectedState()->Position()).length();

        if (len <= distance_threshold) {
          if (weighted) {
            double conf = hypothese[j]->Confidence();

            mC += conf;
            mD += conf * len;
            mI += conf * (objects[i]->Intention()
                == hypothese[j]->mIntention);
          }
          else {
            mC += 1.0;
            mD += len;
            mI += (objects[i]->Intention()
                == hypothese[j]->mIntention);
          }

          foreach_(const Mapping &mapping, mCLEARMapping) {
            if (mapping.first == objects[i]) {
              if (mapping.second != hypothese[j]) {
                mMME += 1.0;

                foreach_(const Mapping &mapping, clear_mapping) {
                  if (mapping.first == objects[i]) {
                    mmes.insert(mapping);
                  }
                }
              }
            }
          }

          clear_mapping.insert(make_pair(objects[i], hypothese[j]));
        }
      }
    }
  }

  foreach_(const Mapping &mapping, mmes) {
    clear_mapping.erase(mapping);
  }

  mCLEARMapping = clear_mapping;

  double g = 0.0, m = 0.0, fp = 0.0;

  foreach_(Human::Ptr &real, mSimulator->mHumans) {
    if (mSimulator->InFieldOfView(real->Position())) {
      g += 1.0;
    }
  }

  assert(mCLEARMapping.size() <= g);
  assert(mCLEARMapping.size() <= identified_humans.size());

  if (weighted) {
    double a = 0.0;
    double e = 0.0;

    foreach_(IdentifiedHuman::Ptr &ih, identified_humans) {
      a += ih->Confidence();
    }

    foreach_(const Mapping &mapping, mCLEARMapping) {
      e += mapping.second->Confidence();
    }

    m = g - e;
    fp = a - e;

    mR += a;
  }
  else {
    m = g - mCLEARMapping.size();
    fp = identified_humans.size() - mCLEARMapping.size();

    mR += identified_humans.size();
  }

  mG += g;
  mM += m;
  mFP += fp;

  foreach_(Human::Ptr &real, mSimulator->mHumans) {
    if (mSimulator->InFieldOfView(real->Position())) {
      shared_ptr<vector2d> track;
      foreach_(const Mapping &mapping, clear_mapping) {
        if (mapping.first == real) {
          track = make_shared<vector2d>(
              mapping.second->ExpectedState()->Position());
          break;
        }
      }

      mTracks[real->mID].push_back(make_pair(real->Position(), track));
    }
  }
}

void Simulator::Human::Predict(double duration)
{
  HumanIntention *intention = 0;
  HumanState hs(*this);

  intention = Intention();
  intention = intention->ChangeIntention(hs, duration);

  SetIntention(intention);

  shared_ptr<HumanAction> action = intention->ChooseAction(hs);

  if (string("motion_model::Stay") == action->GetName()) {
    _motion::Stay().Apply(*this, duration);
  }
  else {
    double power = _motion::RandomPower();
    double angle = _motion::RandomAngle();

    _motion::Dash(power, angle).Apply(*this, duration);
  }
}
