/*
 * main.cpp
 *
 *  Created on: Jan 9, 2014
 *      Author: baj
 */

#include <sys/types.h>
#include <unistd.h>

#include <tracker.h>

#include "task.h"
#include "elevator_entering.h"
#include "tracker.h"
#include "combination.h"
#include "murty.h"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include "popt_pp.h"
#include "terminal_utils.h"
#include "timer.h"
#include <input_stream.h>
#include "params.h"
#include <assert.h>

#include <vector>
#include <list>
#include <cmath>
#include "geometry.h"

#include <boost/foreach.hpp>
#include "helpers.h"
#include "testCalibration.h"
#include "benchmark.h"

#include <boost/program_options.hpp>

using namespace std;
using namespace boost::program_options;

namespace {
const char *date_string()
{
  static char date_time[256];

  time_t t = time(0);
  struct tm* now = localtime(&t);
  strftime(date_time, 200, "%Y-%m-%d-%H-%M-%S", now);

  return date_time;
}

void fitting_observation_data(const string data)
{
  ifstream fin(data.c_str());

  if (!fin.good()) {
    return;
  }

  vector<vector2d> positions;

  vector2d pos;
  while (fin >> pos.x >> pos.y) {
    positions.push_back(pos);
  }
  fin.close();

  vector2d mean(0.0, 0.0);
  foreach_(vector2d &p, positions) {
    mean += p;
  }
  mean /= positions.size();

  STATISTIC::Ptr stat;
  STATISTIC::Create(stat);

  foreach_(vector2d &p, positions) {
    STATISTIC::Add(stat, (p - mean).x);
    STATISTIC::Add(stat, (p - mean).y);
  }

  STATISTIC::Print(stat, "observation error");
}

string RandomString(uint len)
{
  static string charset =
      "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";

  SimpleRNG::ins().RandomSeeding(getpid());

  string result;
  result.resize(len);

  for (uint i = 0; i < len; i++) {
    result[i] = charset[SimpleRNG::ins().GetRand() % charset.length()];
  }

  return result;
}
}

int main(int argc, char** argv)
{
  if (!Params::ins().Parse(argc, argv)) {
    exit(1);
  }

  if (!Params::ins().camera_calibration.empty()
      && Params::ins().benchmark_data.empty()) {
    test_calibration(Params::ins().camera_calibration.c_str());
    exit(0);
  }

  shared_ptr<Task> task = TaskFactory::ins().CreateTask(
      Params::ins().task_name);

  if (!task) {
    TerminalLogger::log() << "Please specify a valid task name." << endl;
    TerminalLogger::log() << "-- Available tasks are: ";
    TaskFactory::ins().PrintTasks(TerminalLogger::log());
    TerminalLogger::log() << endl;

    exit(1);
  }

  Combination::ins();
  Murty::ins();

  const char *log_name =
      Params::ins().log_date? date_string(): DEFAULT_LOG_NAME;

  shared_ptr<HumanTracker> tracker =
      make_shared<HumanTracker>(
          task.get(), true, Params::ins().debug_level, log_name);

  task->Setup(tracker.get());

  TerminalLogger::log() << endl;
  TerminalLogger::log() << "CoBot human tracker node: task name="
      + task->GetName() + "\n\n" << endl;

  string node_name = "Cobot_Human_Tracker_"
      + task->GetName() + "_" + RandomString(2);

  SimpleRNG::ins().RandomSeeding(0);

  if (Params::ins().save_input
      || !Params::ins().load_input.empty()) {
    if (Params::ins().save_input
        && !Params::ins().load_input.empty()) {
      PRINT_ERROR("save_input && !load_input.empty()");
      exit(0);
    }

    if (Params::ins().save_input) {
      string save_name = DEFAULT_LOG_PATH;
      save_name += log_name;
      save_name += ".input";

      InputStream::ins().Save(save_name.c_str());
    }
    else {
      Params::ins().simulation_testing = false;

      string load_name = DEFAULT_LOG_PATH;
      load_name += Params::ins().load_input;

      InputStream::ins().Load(load_name.c_str());
    }
  }

  if (Params::ins().simulation_testing) {
    task->SimulationTest(argc, argv);
  }
  else if (!Params::ins().load_input.empty()) {
    Params::ins().Dump();

    while (Params::ins().runto_step < 0
        || int(tracker->mCurrentStep) <= Params::ins().runto_step) {
      if (InputStream::ins().End()) {
        break;
      }

      Observation::Ptr obs = make_shared<Observation>();
      tracker->Update(obs);
    }
  }
  else if (!Params::ins().benchmark_data.empty()
      && !Params::ins().camera_calibration.empty()) {
    shared_ptr<Simulator> benchmark =
        make_shared<BenchmarkSimulator>(
            task.get(),
            tracker.get(),
            Params::ins().camera_calibration,
            Params::ins().benchmark_data[0],
            Params::ins().benchmark_data[1],
            argc, argv);

    benchmark->Run(argc, argv);
  }

  return 0;
}

