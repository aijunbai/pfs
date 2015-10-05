/*
 * task.cpp
 *
 *  Created on: Jan 10, 2014
 *      Author: baj
 */

#include <cmath>
#include <task.h>
#include "intention.h"
#include "tracker.h"
#include "params.h"
#include "open_area.h"

using namespace std;

Task::Task(string name): mName(name), mTracker(0)
{
  Params::ins().false_rate = 2.0;
  Params::ins().missing_rate = 4.0;
  Params::ins().death_rate = 0.02;
  Params::ins().observation_error = 0.3;
  Params::ins().sigma_power = 1.0;
  Params::ins().max_speed = 2.0;
  Params::ins().max_em_steps = 10;

  Params::ins().observation_proposal_prob = 0.9;
  Params::ins().false_density = 1.0;
  Params::ins().refinement_rate = 0.0;

  Params::ins().max_humans = 5;
  Params::ins().num_particles = 256;

  Params::ins().velocity_augment = true;
  Params::ins().detection_confidence = true;
  Params::ins().detection_orientation = false;
  Params::ins().hierarchical_filters = true;
  Params::ins().gaussian_approximate = true;
}

Task::~Task()
{
}

void Task::SimulationTest(int argc, char **argv)
{
  shared_ptr<Simulator> simulator = CreateSimulator();

  if (simulator) {
    simulator->Run(argc, argv);
  }
}

void Task::Setup(HumanTracker *tracker)
{
  mTracker = tracker;

  if (Params::ins().intention_mode >= 0) {
    RegisterIntentions();
  }
  else {
    IntentionFactory::ins().Register( //random walk model only
        make_shared<OpenAreaTask::Move>(GetTracker()));
  }
}

TaskFactory::TaskFactory()
{
}

TaskFactory::~TaskFactory()
{
}

TaskFactory &TaskFactory::ins()
{
  static TaskFactory task_factory;
  return task_factory;
}

bool TaskFactory::Register(const string &name, TaskCreator creator)
{
  mTaskMap[name] = creator;

  return true;
}

shared_ptr<Task> TaskFactory::CreateTask(const string &name)
{
  TerminalLogger::log() << "Finding task: " << name << endl;

  if (mTaskMap.count(name)) {
    return mTaskMap[name]();
  }

  return shared_ptr<Task>();
}

void TaskFactory::PrintTasks(ostream &os)
{
  for (HashMap<string, TaskCreator>::iterator it = mTaskMap.begin(); it != mTaskMap.end(); ++it) {
    os << it->first << " ";
  }
}

