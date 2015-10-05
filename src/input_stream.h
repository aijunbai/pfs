/*
 * time_line.h
 *
 *  Created on: Jan 28, 2014
 *      Author: baj
 */

#ifndef TIME_LINE_H_
#define TIME_LINE_H_

#include <vector>
#include <stack>

#include "geometry.h"
#include "observation.h"

struct RobotPose;

/**
 * save all inputs (including time) for debug purposes
 */
class InputStream
{
  InputStream();
  virtual ~InputStream();

public:
  static InputStream &ins();

  void Load(std::string file_name);
  void Save(std::string file_name);

  void Filter(
      RobotPose &robot_pose,
      std::vector<Detection::Ptr> &pos,
      double &duration);

  bool DebugMode();
  bool End();

private:
  std::stack<std::vector<Detection::Ptr> > mObservations;
  std::stack<std::pair<vector2d, double> > mRobotPoses;
  std::stack<double> mDuration;
  std::string mSaveName;
  bool mDebugMode;
};

#endif /* TIME_LINE_H_ */
