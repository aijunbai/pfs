/*
 * logger.h
 *
 *  Created on: Sep 11, 2010
 *      Author: baj
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using boost::shared_ptr;
using boost::make_shared;

#define PRINT_ERROR(error) \
  do { \
    TerminalLogger::log() << __FILE__ << ":" << __LINE__ << " : " << error << std::endl; \
  } while(0)

#define PRINT_VALUE(x) \
  do { \
  TerminalLogger::log() << #x " = '" << x << "'" << std::endl; \
  } while(0)

/**
 * RCG format logger (should be played with rcg_player)
 */
class RCGLogger {
public:
  typedef shared_ptr<RCGLogger> Ptr;

  enum Color {
    Orange,
    Cyan,
    Purple,
    Yellow,
    Olive,
    Navy,
    Green,
    Gray,
    Red,
    Blue,
    White,
    Black,

    Color_Max
  };

  struct Vector {
    double x_;
    double y_;

    Vector(const double &x, const double &y): x_(x), y_(y) {

    }

    Vector operator-() const { return Vector(-x_, -y_); }
    Vector operator+(const Vector &a) const { return Vector(x_ + a.x_, y_ + a.y_); }
    Vector operator-(const Vector &a) const { return Vector(x_ - a.x_, y_ - a.y_); }
    Vector operator*(const double &a) const { return Vector(x_ * a, y_ * a); }
    Vector operator/(double a) const { return Vector(x_ / a, y_ / a); }

    const double &X() const { return x_; }
    const double &Y() const { return y_; }
  };

  class Rectangular
  {
  public:
    Rectangular(): left_(0.0), right_(0.0), top_(0.0), bottom_(0.0) {}
    Rectangular(const double &left, const double &right, const double &top, const double &bottom): left_(left), right_(right), top_(top), bottom_(bottom) {}

    const double &Left() const { return left_; }
    const double &Right() const { return right_; }
    const double &Top() const { return top_; }
    const double &Bottom() const { return bottom_; }

    void SetLeft(const double &left) { left_ = left; }
    void SetRight(const double &right) { right_ = right; }
    void SetTop(const double &top) { top_ = top; }
    void SetBottom(const double &bottom) { bottom_ = bottom; }

    Vector TopLeftCorner() const { return Vector(left_, top_); }
    Vector TopRightCorner() const { return Vector(right_, top_); }
    Vector BottomLeftCorner() const { return Vector(left_, bottom_); }
    Vector BottomRightCorner() const { return Vector(right_, bottom_); }

  public:
    double left_;
    double right_;
    double top_;
    double bottom_;
  };

private:
  struct ItemShape {
    Color line_color;

    const char *color() const {
      switch (line_color) {
      case Red: return "red";
      case Blue: return "blue";
      case Green: return "green";
      case Navy: return "navy";
      case Orange: return "orange";
      case Cyan: return "cyan";
      case Purple: return "purple";
      case White: return "white";
      case Black: return "black";
      case Yellow: return "yellow";
      case Olive: return "olive";
      case Gray: return "gray";
      default: return "black";
      }

      return "black";
    }

    ItemShape (Color color){
      line_color = color;
    }
  };

  struct PointShape: public ItemShape {
    double x, y;
    std::string comment;

    PointShape (double x_, double y_, Color color, const char* cmt = 0): ItemShape(color), x(x_), y(y_) {
      if (cmt) {
        comment.assign(std::string("@") + cmt);
      }
    };

    friend std::ostream& operator<<(std::ostream &os, PointShape &point) {
      return os << "(point " << point.x << ' ' << point.y << ' ' << "\"" << point.color() << "\" "<<point.comment<<")";
    }
  };

  struct LineShape: public ItemShape {
    double x1, y1;
    double x2, y2;

    LineShape (double x1_, double y1_, double x2_, double y2_, Color color):
       ItemShape(color), x1(x1_), y1(y1_), x2(x2_), y2(y2_)
    {
    };

    friend std::ostream& operator<<(std::ostream &os, const LineShape &line) {
      return os << "(line " << line.x1 << ' ' << line.y1 << ' ' << line.x2 << ' ' << line.y2 << " \"" << line.color() << "\")";
    }
  };

  struct CircleShape: public ItemShape {
    double x, y;
    double radius;

    CircleShape (double x_, double y_, double r, Color color): ItemShape(color), x(x_), y(y_), radius(r) {
    };

    friend std::ostream& operator<<(std::ostream &os, CircleShape &circle) {
      return os << "(circle " << circle.x << ' ' << circle.y << ' ' << circle.radius << " \"" << circle.color() << "\")";
    }
  };

public:
  void AddText(double x, double y, Color color, const char* comment) {
    points_.push_back(PointShape(x / scale_x_, y / scale_y_, color, comment));
  }

  void AddPoint(double x, double y, Color color = Red, const char* comment = 0) {
    points_.push_back(PointShape(x, y, color, comment));
  }

  void AddLine(double x1, double y1, double x2, double y2, Color color = Yellow) {
    lines_.push_back(LineShape(x1, y1, x2, y2, color));
  }

  void AddCircle(double x, double y, const double &radius, Color color = White) {
    circles_.push_back(CircleShape(x, y, radius, color));
  }

public:
  RCGLogger(const char *file_name): time_(0), focus_(0.0, 0.0), scale_x_(1.0), scale_y_(1.0) {
    std::string file = file_name;
    file += ".rcg";

    fout.open(file.c_str());
  }

  ~RCGLogger() {
    fout.close();
  }

  void Flush();

  void LogLine(double x1, double y1, double x2, double y2, Color color, const char* comment = 0);
  void LogCircle(double x, double y, double r, Color color);
  void LogRectangular(const double left, const double right, const double top, const double bottom, Color color);

  void Scale(double x, double y);
  void Focus(double x, double y);

private:
  std::ofstream fout;
  int time_;

  std::vector<PointShape> points_;
  std::vector<LineShape> lines_;
  std::vector<CircleShape> circles_;

  Vector focus_;
  double scale_x_;
  double scale_y_;
};

class StreamLogger
{
public:
  typedef std::string (*FunPtr)();

private:
  class Stream: public std::ostream
  {
  public:
    class StreamBuf: public std::stringbuf
    {
      FunPtr mBeginFun;
      FunPtr mEndFun;
      std::ostream&   mStream;
    public:
      StreamBuf(std::ostream& os, FunPtr begin, FunPtr end):
        mBeginFun(begin),
        mEndFun(end),
        mStream(os)
    {

    }

      virtual int sync ()
      {
        if (str().empty()) {
          return 0;
        }

        if (mBeginFun) {
          mStream << mBeginFun();
        }

        mStream  << str();

        if (mEndFun) {
          mStream << mEndFun();
        }

        mStream.flush();
        str("");

        return 0;
      }
    };

    Stream(std::ostream &os, FunPtr begin, FunPtr end):
      std::ostream(&mBuffer),
      mBuffer(os, begin, end)
    {

    }

  private:
    StreamBuf mBuffer;
  };

  shared_ptr<Stream> mStream;

public:
  StreamLogger();
  virtual ~StreamLogger();

  void Open(
      std::ostream &os,
      StreamLogger::FunPtr begin,
      StreamLogger::FunPtr end);

  bool good() const {
    return mStream != 0;
  }

  operator bool() const {
    return good();
  }

  template<typename T>
  StreamLogger& operator<<(const T& value)
  {
    if (good())
    {
      *mStream << value;
    }
    return *this;
  }

  StreamLogger& operator<<(std::ostream& (*fun)(std::ostream&))
  {
    if (good())
    {
      *mStream << fun;
    }
    return *this;
  }

  StreamLogger& operator<<(std::ios_base& (*fun)(std::ios_base&))
  {
    if (good())
    {
      *mStream << fun;
    }
    return *this;
  }

  std::ostream &stream() {
    return *mStream;
  }
};

class TerminalLogger: public StreamLogger {
private:
  TerminalLogger();
  ~TerminalLogger();

public:
  static std::ostream &log();
};

#endif /* LOGGER_H_ */
