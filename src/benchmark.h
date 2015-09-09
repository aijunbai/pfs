/*
 * bechmark.h
 *
 *  Created on: Jun 4, 2014
 *      Author: baj
 */

#ifndef BENCHMARK_H_
#define BENCHMARK_H_

#include "common.h"
#include "simulator.h"
#include "cameraModel.h"
#include "xmlUtil.h"
#include "tracker.h"

#include <QtCore>
#include <QtGui>
#include <QObject>
#include <QLabel>
#include <QPixmap>
#include <QResizeEvent>

#define XML_TAG_DATASET BAD_CAST"dataset"
#define XML_TAG_NAME BAD_CAST"name"

#define XML_TAG_FRAME BAD_CAST"frame"
#define XML_TAG_NUMBER BAD_CAST"number"
#define XML_TAG_OBJECTLIST BAD_CAST"objectlist"

#define XML_TAG_OBJECT BAD_CAST"object"
#define XML_TAG_CONFIDENCE BAD_CAST"confidence"
#define XML_TAG_ID BAD_CAST"id"

#define XML_TAG_BOX BAD_CAST"box"
#define XML_TAG_H BAD_CAST"h"
#define XML_TAG_W BAD_CAST"w"
#define XML_TAG_XC BAD_CAST"xc"
#define XML_TAG_YC BAD_CAST"yc"

class AspectRatioPixmapLabel : public QLabel
{
    Q_OBJECT
public:
    explicit AspectRatioPixmapLabel(QWidget *parent = 0);
    virtual int heightForWidth( int width ) const;
    virtual QSize sizeHint() const;

public slots:
    void setPixmap ( const QPixmap & );
    void resizeEvent(QResizeEvent *);

private:
    QPixmap pix;
};

class BenchmarkSimulator;

class Displayer: public AspectRatioPixmapLabel
{
  Q_OBJECT
public:
  Displayer(BenchmarkSimulator *simulator):
    mSimulator(simulator)
  {

  }

public slots:
  void update();

private:
  void DrawField(QPainter &painter, int p, int view);
  void DrawDetection(
      vector2d &pos, double h, double w, double conf,
      QColor &color, double width, bool fill, QPainter &painter);
  void DrawParticles(
      HumanStatePool &states, double conf,
      QColor &color, double width, QPainter &painter);

private:
  BenchmarkSimulator *mSimulator;
};

class Waiter: public QThread {
  Q_OBJECT

public:
  Waiter(ThreadCondition &signal):
    QThread(),
    mSignal(signal)
  {

  }

  void run();

private:
  ThreadCondition &mSignal;

signals:
  void updated();
};

class BenchmarkSimulator: public Simulator {
public:
  BenchmarkSimulator(
      Task *task,
      HumanTracker *tracker,
      const std::string camera_calibration,
      const std::string detections,
      const std::string ground_truth,
      int argc,
      char **argv);

  virtual ~BenchmarkSimulator();

  virtual void Initialize(double expected_humans);
  virtual void Simulate(double duration, double birth_rate, double death_rate);
  virtual void GenerateObservations(Observation &obs, double duration);
  virtual bool End();
  virtual void CheckConsistence(Observation &obs);
  virtual void LogRCG();

  bool InFieldOfView(const vector2d& v) {
    return mGroundTruths.InFieldOfView(v);
  }

  virtual void Display();

private:
  struct BenchmarkData {
    struct Object {
      int frame;
      int id;
      double confidence;
      double ih;
      double iw;
      double rh;
      double rw;
      double xc;
      double yc;

      vector2d position;

      Object(): frame(-1), id(-1), confidence(-1.0),
          ih(0), iw(0), rh(0), rw(0), xc(0), yc(0) {

      }
    };

    bool fromXml(std::istream& is, Etiseo::CameraModel &camera);
    void InitMetaData();

    int TotalSize() const {
      int ret = 0;

      foreach_(const std::list<Object> &i, mData) {
        ret += i.size();
      }

      return ret;
    }

    int Frames() const {
      return mData.size();
    }

    bool InFieldOfView(const vector2d& v) {
      if (mField) {
        return v.x >= mField->left() && v.x <= mField->right() &&
            v.y >= mField->top() && v.y <= mField->bottom();
      }

      return true;
    }

    std::vector<std::list<Object> > mData;
    std::string mName;

    vector2d mCenter;
    shared_ptr<QRectF> mField;
    int mStartNumber;
    std::string mRawDataPath;
    std::string mFormat;
  };

public:
  BenchmarkData mDetections;
  BenchmarkData mGroundTruths;
  Etiseo::CameraModel mCamera;
  shared_array<ThreadCondition> mSignals;
};

#endif /* BECHMARK_H_ */
