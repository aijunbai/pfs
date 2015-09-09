/*
 * time_line.cpp
 *
 *  Created on: Jan 28, 2014
 *      Author: baj
 */

#include <fstream>
#include <input_stream.h>
#include "common.h"
#include "logger.h"
#include <unistd.h>

using namespace std;

InputStream::InputStream(): mDebugMode(false)
{

}

InputStream::~InputStream()
{
  if (!mSaveName.empty()) {
    ofstream fout(mSaveName.c_str());

    if (!fout.good()) {
      PRINT_ERROR("Open " << mSaveName << " error!");
      return;
    }

    fout << mObservations.size() << endl;
    while (!mObservations.empty()) {
      fout << mObservations.top().size() << endl;
      foreach_(Detection::Ptr &det, mObservations.top()) {
        fout << det->mPosition.x << " "
            << det->mPosition.y << " "
            << det->mHeight[0] << " "
            << det->mWidth[0] << " "
            << det->mHeight[1] << " "
            << det->mWidth[1] << " "
            << det->mConfidence << endl;
      }
      mObservations.pop();
    }

    fout << mRobotPoses.size() << endl;
    while (!mRobotPoses.empty()) {
      std::pair<vector2d, double> pose = mRobotPoses.top();
      fout << pose.first.x << " " << pose.first.y << " " << pose.second << endl;
      mRobotPoses.pop();
    }

    fout << mDuration.size() << endl;
    while (!mDuration.empty()) {
      double duration = mDuration.top();
      fout << duration << " " << endl;
      mDuration.pop();
    }

    fout.close();
  }
}

InputStream &InputStream::ins()
{
  static InputStream time_line;

  return time_line;
}

void InputStream::Load(string file_name)
{
  mDebugMode = true;

  if (!file_name.empty()) {
    ifstream fin(file_name.c_str());

    if (!fin.good()) {
      PRINT_ERROR("Open " << file_name << " error!");
      return;
    }

    int size = 0;

    fin >> size;
    for (int i = 0; i < size; ++i) {
      int obs = 0;
      Detection::Ptr det = make_shared<Detection>();
      vector<Detection::Ptr> observation;

      fin >> obs;
      for (int j = 0; j < obs; ++j) {
        fin >> det->mPosition.x >>
          det->mPosition.y >>
          det->mHeight[0] >>
          det->mWidth[0] >>
          det->mHeight[1] >>
          det->mWidth[1] >>
          det->mConfidence;
        observation.push_back(det);
      }
      mObservations.push(observation);
    }

    fin >> size;
    for (int i = 0; i < size; ++i) {
      vector2d pos;
      double angle;
      fin >> pos.x >> pos.y >> angle;
      mRobotPoses.push(make_pair(pos, angle));
    }

    fin >> size;
    for (int i = 0; i < size; ++i) {
      double duration;
      fin >> duration;
      mDuration.push(duration);
    }


    fin.close();
  }
}

void InputStream::Save(string file_name)
{
  mSaveName = file_name;
}

bool InputStream::DebugMode()
{
  return mDebugMode;
}

bool InputStream::End()
{
  return mDebugMode &&
      (mDuration.empty() || mObservations.empty() || mRobotPoses.empty());
}

void InputStream::Filter(
    RobotPose &robot_pose,
    vector<Detection::Ptr> &pos,
    double &duration)
{
  if (mDebugMode) {
    if (End()) {
      PRINT_ERROR("input stream error!");
      return;
    }

    pos = mObservations.top();
    robot_pose.mPosition = mRobotPoses.top().first;
    robot_pose.mAngle = mRobotPoses.top().second;
    duration = mDuration.top();

    mRobotPoses.pop();
    mObservations.pop();
    mDuration.pop();
  }
  else if (!mSaveName.empty()) {
    mObservations.push(pos);
    mRobotPoses.push(make_pair(robot_pose.mPosition, robot_pose.mAngle));
    mDuration.push(duration);
  }
}
