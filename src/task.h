/*
 * task.h
 *
 *  Created on: Jan 10, 2014
 *      Author: baj
 */

#ifndef TASK_H_
#define TASK_H_

#include <string>
#include <iostream>
#include "logger.h"
#include "intention.h"
#include "simulator.h"
#include "tracker.h"
#include "gvector.h"

class HumanTracker;
class Task;

class TaskFactory
{
private:
  TaskFactory();
  virtual ~TaskFactory();

public:
  typedef shared_ptr<Task> (*TaskCreator)();

  static TaskFactory &ins();

  bool Register(const std::string &name, TaskCreator creator);
  shared_ptr<Task> CreateTask(const std::string &name);

  void PrintTasks(std::ostream &os);

private:
  HashMap<std::string, TaskCreator> mTaskMap;
};

class Task
{
public:
  Task(std::string name);
  virtual ~Task();

  std::string &GetName() {
    return mName;
  }

  HumanTracker *GetTracker() {
    return mTracker;
  }

  virtual void Setup(HumanTracker *tracker);

  virtual void RegisterIntentions() = 0;
  virtual vector2d NewHumanPosition() = 0;
  virtual shared_ptr<Simulator> CreateSimulator() = 0;

  void SimulationTest(int argc, char **argv);

  template <class TaskDerived >
  static shared_ptr<Task> Creator() { return make_shared<TaskDerived>(); }

  template <class TaskDerived>
  static bool Register(const std::string &name) {
    return TaskFactory::ins().Register(name, Creator<TaskDerived>);
  }

private:
  std::string mName;
  HumanTracker *mTracker;
};


#endif /* TASK_H_ */
