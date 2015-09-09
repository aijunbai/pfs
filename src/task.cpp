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

