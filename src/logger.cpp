/*
 * logger.cpp
 *
 *  Created on: Sep 11, 2010
 *      Author: baj
 */

#include "logger.h"
#include "common.h"
#include "terminal_utils.h"
#include "tracker.h"

using namespace std;

void RCGLogger::LogLine(double x1, double y1, double x2, double y2, Color color, const char* comment)
{
  AddPoint(x2, y2, color, comment);
  AddLine(x1, y1, x2, y2, color);
}

void RCGLogger::LogCircle(double x, double y, double r, Color color)
{
  AddCircle(x, y, r, color);
}

void RCGLogger::LogRectangular(const double left, const double right, const double top, const double bottom, RCGLogger::Color color)
{
  AddLine(left, top, right, top, color);
  AddLine(left, top, left, bottom, color);
  AddLine(left, bottom, right, bottom, color);
  AddLine(right, bottom, right, top, color);
}

void RCGLogger::Flush()
{
  if (!fout.good()) return;

  fout << "(scale " << time_ << ' ' << scale_x_ << ' ' << scale_y_ <<")\n";
  fout << "(focus " << time_ << ' ' << focus_.x_ << ' ' << focus_.y_ << ")\n";

  if (!points_.empty()) {
    for (std::vector<PointShape>::iterator it = points_.begin(); it != points_.end(); it++)
    {
      fout << "(draw " << time_ << ' '<< *it <<")\n";
    }
    points_.clear();
  }

  if (!lines_.empty()) {
    for (std::vector<LineShape>::iterator it = lines_.begin(); it != lines_.end(); it++)
    {
      fout << "(draw " << time_ << ' '<< *it <<")\n";
    }
    lines_.clear();
  }

  if (!circles_.empty()) {
    for (std::vector<CircleShape>::iterator it = circles_.begin(); it != circles_.end(); it++)
    {
      fout << "(draw " << time_ << ' '<< *it <<")\n";
    }
    circles_.clear();
  }

  time_ += 1;
}

void RCGLogger::Scale(double x, double y)
{
  scale_x_ = x;
  scale_y_ = y;
}

void RCGLogger::Focus(double x, double y)
{
  focus_ = Vector(x, y);
}

StreamLogger::StreamLogger(): mStream()
{

}

void StreamLogger::Open(
    std::ostream &os,
    StreamLogger::FunPtr begin,
    StreamLogger::FunPtr end)
{
  mStream = shared_ptr<Stream>(new Stream(os, begin, end));
}

StreamLogger::~StreamLogger()
{
  if (good()) {
    mStream->flush();
  }
}

namespace {
string TerminalLoggerBegin()
{
  ColourTerminal(TerminalUtils::TERMINAL_COL_BLUE,
                 TerminalUtils::TERMINAL_COL_BLACK,
                 TerminalUtils::TERMINAL_ATTR_BRIGHT);

  stringstream ss;
  ss << "HumanTracker @ "
      << HumanTracker::mCurrentStep << " -: ";

  return ss.str();
}

string TerminalLoggerEnd()
{
  ResetTerminal();
  return string();
}
}

TerminalLogger::TerminalLogger()
{
  Open(std::cout, TerminalLoggerBegin, TerminalLoggerEnd);
}

TerminalLogger::~TerminalLogger()
{
}

ostream &TerminalLogger::log()
{
  static TerminalLogger terminal_logger;
  return terminal_logger.stream();
}
