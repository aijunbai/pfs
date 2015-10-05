/*
 * benchmark.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: baj
 */

#include <list>
#include <benchmark.h>
#include "tracker.h"
#include "xmlUtil.h"
#include "state.h"

#include <pthread.h>
#include <iomanip>
#include <sys/time.h>

#include <libxml/xmlwriter.h>
#include <libxml/xmlreader.h>

using namespace std;
using namespace Etiseo;

namespace {

struct InputArgs{
  int argc;
  char **argv;
  BenchmarkSimulator *simulator;
};

void *StartQAppThread(void *threadArg)
{
  InputArgs *args = (struct InputArgs*) threadArg;
  QApplication app(args->argc, args->argv);

  Waiter waiter(args->simulator->mSignals[0]);
  Displayer displayer(args->simulator);

  QObject::connect(
      &waiter, SIGNAL(updated()),
      &displayer, SLOT(update()));

  waiter.start();

  QPalette pal = displayer.palette();
  pal.setColor(displayer.backgroundRole(), Qt::black);
  displayer.setPalette(pal);

  args->simulator->mSignals[2].Set();
  app.exec();

  pthread_exit(NULL);
}

void StartAppThread(InputArgs &args)
{
  pthread_t thread;
  pthread_create(&thread, NULL, StartQAppThread, (void*)&args);
}

}

AspectRatioPixmapLabel::AspectRatioPixmapLabel(QWidget *parent) :
    QLabel(parent)
{
    this->setMinimumSize(1,1);
}

void AspectRatioPixmapLabel::setPixmap ( const QPixmap & p)
{
    pix = p;
    resizeEvent(0);
}

int AspectRatioPixmapLabel::heightForWidth( int width ) const
{
    return ((qreal)pix.height()*width)/pix.width();
}

QSize AspectRatioPixmapLabel::sizeHint() const
{
    int w = this->width();
    return QSize( w, heightForWidth(w) );
}

void AspectRatioPixmapLabel::resizeEvent(QResizeEvent *)
{
    QLabel::setPixmap(pix.scaled(
        this->size(),
        Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void Displayer::DrawField(QPainter &painter, int p, int view)
{
  if (mSimulator->mGroundTruths.mField) {
    QPointF topleft = mSimulator->mCamera.GroundToImage(
        mSimulator->mGroundTruths.mField->left(),
        mSimulator->mGroundTruths.mField->top());
    QPointF topright = mSimulator->mCamera.GroundToImage(
        mSimulator->mGroundTruths.mField->right(),
        mSimulator->mGroundTruths.mField->top());
    QPointF bottomleft = mSimulator->mCamera.GroundToImage(
        mSimulator->mGroundTruths.mField->left(),
        mSimulator->mGroundTruths.mField->bottom());
    QPointF bottomright = mSimulator->mCamera.GroundToImage(
        mSimulator->mGroundTruths.mField->right(),
        mSimulator->mGroundTruths.mField->bottom());

    QColor color(Qt::black);
    color.setAlphaF(0.25);

    painter.setPen (Qt::NoPen);

    QPainterPath path;

    path.moveTo(painter.window().topLeft());
    path.lineTo(painter.window().topRight());
    path.lineTo(topright);
    path.lineTo(bottomright);
    path.lineTo(bottomleft);
    path.lineTo(painter.window().topLeft());

    painter.fillPath(path, QBrush(color));

    path.moveTo(painter.window().bottomLeft());
    path.lineTo(bottomleft);
    path.lineTo(topleft);
    path.lineTo(topright);
    path.lineTo(painter.window().bottomRight());
    path.lineTo(painter.window().bottomLeft());

    painter.fillPath(path, QBrush(color));
  }

  painter.setPen(QPen(Qt::green, 2.0));

  QFont font;
  font.setBold(true);
  font.setPixelSize(25);
  painter.setFont(font);
  painter.drawText(10, 30, QString("Frame id %1").arg(
      mSimulator->mGroundTruths.mStartNumber + HumanTracker::mCurrentStep - 1));

  if (Params::ins().side_view) {
    font.setPixelSize(20);
    painter.setFont(font);
    painter.drawText(
        painter.window().width() - 35, painter.window().height() - 15,
        QString("(%1)").arg(view));
  }
}

void Displayer::DrawDetection(
    vector2d &pos, double h, double w, double conf,
    QColor &color, double width, bool fill, QPainter &painter)
{
  QPointF point = mSimulator->mCamera.GroundToImage(pos.x, pos.y);
  QRectF rect(point.x() - 0.5 * w, point.y() -h, w, h);

  color.setAlphaF(0.9);

  painter.setPen(QPen(color, width));
  painter.drawRect(rect);

  if (fill) {
    color.setAlphaF(0.1);
    painter.fillRect(rect, QBrush(color));
  }
}

void Displayer::DrawParticles(
      HumanStatePool &states, double conf,
      QColor &color, double width, QPainter &painter)
{
  color.setAlphaF(1.0);
  painter.setPen(QPen(color, width));

  foreach_(HumanState *h, states) {
    QPointF point = mSimulator->mCamera.GroundToImage(h->mPosition.x, h->mPosition.y);
    painter.drawPoint(point);
  }
}

void Displayer::update()
{
  stringstream ss;
  ss << mSimulator->mGroundTruths.mRawDataPath << setw(4) << setfill('0')
      << mSimulator->mGroundTruths.mStartNumber + HumanTracker::mCurrentStep - 1
      << mSimulator->mGroundTruths.mFormat;

  shared_ptr<QPixmap> pixmap[5];
  shared_ptr<QPainter> painter[5];

  for (int i = 0; i < (Params::ins().side_view? 4: 1); ++i) {
    pixmap[i] = make_shared<QPixmap>();
    pixmap[i]->load(QString::fromStdString(ss.str()));

    if (pixmap[i]->isNull()) {
      PRINT_ERROR("Open " << ss.str() << " Error!");
      mSimulator->mSignals[1].Set();
      return;
    }
  }

  if (Params::ins().side_view) {
    pixmap[4] = make_shared<QPixmap>(
        pixmap[0]->width() * 2 + 1,
        pixmap[0]->height() * 2 + 1);
  }
  else {
    pixmap[4] = make_shared<QPixmap>(pixmap[0]->width(), pixmap[0]->height());
  }

  for (int p = 0; p < 5; ++p) {
    if (pixmap[p]) {
      painter[p] = make_shared<QPainter>(pixmap[p].get());
      painter[p]->setRenderHints(
          QPainter::Antialiasing |
          QPainter::HighQualityAntialiasing |
          QPainter::TextAntialiasing |
          QPainter::SmoothPixmapTransform);
    }
  }

  int views[] = {3, 4, 1, 2};

  //2(1) | 3(2)
  //-----------
  //1(4) | 0(1)

  for (int p = 0; p < (Params::ins().side_view? 4: 1); ++p) {
    DrawField(*painter[p], p, views[p]);

    if (p == 1 || p == 2) {
      foreach_(const Simulator::Human::Ptr &human, mSimulator->mHumans) {
        int n = human->mTrajectory.size();

        for (int i = 0; i < n - 1; ++i) {
          QColor color = QColor(Qt::white);

          vector2d from = human->mTrajectory.at(i);
          vector2d to = human->mTrajectory.at(i+1);

          QPointF point1 = mSimulator->mCamera.GroundToImage(from.x, from.y);
          QPointF point2 =mSimulator-> mCamera.GroundToImage(to.x, to.y);

          double alpha = pow(0.99, n - 2 - i);

          color.setAlphaF(alpha);
          painter[p]->setPen(QPen(color, 2.5));
          painter[p]->drawLine(QLineF(point1, point2));
        }
      }
    }

    if (p == 0 || p == 3) {
      foreach_(Detection::Ptr &det,
               mSimulator->mTracker->CurrentObservation().mDetections) {
        QColor color = QColor(Qt::white);

        DrawDetection(
            det->mPosition, det->mHeight[0], det->mWidth[0], det->mConfidence,
            color, 2.5, true, *painter[p]);
      }
    }

    if (p == 0 || p == 1) {
      foreach_(IdentifiedHuman::Ptr &ih, mSimulator->mTracker->IdentifiedHumans()) {
        if (ih->Confidence() < Params::ins().report_threshold) {
          continue;
        }

        QColor color = ih->DisplayingColor();
        DrawParticles(ih->mStatePool, ih->Confidence(), color, 2.5, *painter[p]);

        int n = ih->mTrajectory.size();

        for (int i = 0; i < n - 1; ++i) {
          HumanState::Ptr &state = ih->mTrajectory.at(i).state;
          HumanState::Ptr &next_state = ih->mTrajectory.at(i+1).state;

          vector2d &from = state->Position();
          vector2d &to = next_state->Position();

          QPointF point1 = mSimulator->mCamera.GroundToImage(from.x, from.y);
          QPointF point2 =mSimulator-> mCamera.GroundToImage(to.x, to.y);

          double alpha = pow(0.99, n - 2 - i);

          if (Params::ins().report_threshold < 0.0) {
            alpha *= ih->mTrajectory.at(i).confidence;
          }

          color.setAlphaF(alpha);
          painter[p]->setPen(QPen(color, 2.5));
          painter[p]->drawLine(QLineF(point1, point2));
        }
      }
    }
  }

  if (Params::ins().side_view) {
    double h = pixmap[0]->height();
    double w = pixmap[0]->width();

    painter[4]->drawPixmap(QPointF(0, 0), *pixmap[2]);
    painter[4]->drawPixmap(QPointF(w+1, 0), *pixmap[3]);
    painter[4]->drawPixmap(QPointF(0, h+1), *pixmap[1]);
    painter[4]->drawPixmap(QPointF(w+1, h+1), *pixmap[0]);

    painter[4]->setPen(QPen(Qt::black, 2.0));
    painter[4]->drawLine(w, 0, w, 2 * h);
    painter[4]->drawLine(0, h, 2 * w, h);
  }
  else {
    painter[4]->drawPixmap(QPointF(0.0, 0.0), *pixmap[0]);
  }

  QDir dir(QString("/tmp/baj/" +
                   QString::fromStdString(
                       mSimulator->mGroundTruths.mName + "/")));

  ss.str("");
  ss << dir.path().toStdString() << "/frame_" << setw(4) << setfill('0')
              << mSimulator->mGroundTruths.mStartNumber + HumanTracker::mCurrentStep - 1
              << ".png";

  dir.mkpath(".");
  pixmap[4]->save(QString::fromStdString(ss.str()), "PNG");

  setPixmap(*pixmap[4]);

  QLabel::update();
  QLabel::show();

  mSimulator->mSignals[1].Set();
}

void Waiter::run()
{
  while (1) {
    mSignal.Wait();
    emit updated();
  }
}

BenchmarkSimulator::BenchmarkSimulator(
    Task *task,
    HumanTracker *tracker,
    const std::string camera_calibration,
    const std::string detections,
    const std::string ground_truth,
    int argc,
    char **argv): Simulator(task, tracker)
{
  mSignals = shared_array<ThreadCondition>(new ThreadCondition[3]);

  Etiseo::UtilXml::Init();

  std::ifstream is;
  is.open(camera_calibration.c_str());
  mCamera.fromXml(is);
  is.close();

  is.open(detections.c_str());
  mDetections.fromXml(is, mCamera);
  is.close();
  is.open(ground_truth.c_str());
  mGroundTruths.fromXml(is, mCamera);
  is.close();

  PRINT_VALUE(mDetections.Frames());
  PRINT_VALUE(mGroundTruths.Frames());

  PRINT_VALUE(mDetections.TotalSize());
  PRINT_VALUE(mGroundTruths.TotalSize());

  if (mGroundTruths.mName == "PETS-S2L1") {
    Params::ins().false_rate = 6.0;
    Params::ins().missing_rate = 2.0;
    Params::ins().observation_proposal_prob = 0.9;
    Params::ins().observation_error = 0.5;
    Params::ins().false_density = 0.5;
    Params::ins().death_rate = 0.02;
    Params::ins().refinement_rate = 0.0;
  }
  else if (mGroundTruths.mName == "TUD-stadtmitte") {
    Params::ins().false_rate = 7.5;
    Params::ins().missing_rate = 4.0;
    Params::ins().observation_proposal_prob = 0.9;
    Params::ins().observation_error = 0.6;
    Params::ins().false_density = 0.8;
    Params::ins().death_rate = 0.01;
    Params::ins().refinement_rate = 0.01;
  }
  else {
    Params::ins().false_rate = 6.0;
    Params::ins().missing_rate = 2.0;
    Params::ins().observation_proposal_prob = 0.9;
    Params::ins().observation_error = 0.5;
    Params::ins().false_density = 0.5;
    Params::ins().death_rate = 0.02;
    Params::ins().refinement_rate = 0.01;
  }

  Params::ins().view_width = 360.0;
  Params::ins().view_length = 1000.0;
  Params::ins().max_humans = 15;

  Params::ins().num_particles = 128;
  Params::ins().position_kernel_size = 256;
  Params::ins().intention_mode = -1;

  Params::ins().velocity_augment = true;
  Params::ins().detection_confidence = true;
  Params::ins().detection_orientation = false;
  Params::ins().hierarchical_filters = true;
  Params::ins().gaussian_approximate = true;

  if (Params::ins().interface) {
    static InputArgs args = {argc, argv, this};

    StartAppThread(args);
    mSignals[2].Wait();
  }
}

BenchmarkSimulator::~BenchmarkSimulator()
{
}

void BenchmarkSimulator::Initialize(double expected_humans)
{
  mHumans.clear();
}

void BenchmarkSimulator::CheckConsistence(Observation &obs)
{

}

bool BenchmarkSimulator::End()
{
  return mTracker->mCurrentStep >= mGroundTruths.mData.size();
}

void BenchmarkSimulator::Simulate(
    double duration, double birth_rate, double death_rate)
{
  HashMap<int, BenchmarkData::Object*> objects;

  foreach_(
      BenchmarkData::Object &obj,
      mGroundTruths.mData.at(mTracker->mCurrentStep)) {
    objects[obj.id] = &obj;
  }

  {
    list<Human::Ptr>::iterator it = mHumans.begin();
    while (it != mHumans.end()) {
      if (!objects.count((*it)->mID)) {
        it = mHumans.erase(it);
      }
      else {
        ++it;
      }
    }
  }

  foreach_(Human::Ptr &human, mHumans) {
    if (objects.count(human->mID)) {
      human->SetPosition(objects[human->mID]->position);
      objects[human->mID] = 0;
    }
    else {
      assert(0);
    }
  }

  for (HashMap<int, BenchmarkData::Object*>::iterator it = objects.begin();
      it != objects.end(); ++it) {
    if (it->second) {
      Simulator::Human::Ptr human = make_shared<Simulator::Human>(
          it->second->position, it->second->position,
          static_cast<HumanIntention*>(0));
      human->SetID(it->second->id);

      mHumans.push_back(human);
    }
  }

  RobotPose pose;

  pose.mPosition = mGroundTruths.mCenter;
  pose.mAngle = 0.0;

  mTracker->UpdateRobotPose(pose);
}

void BenchmarkSimulator::GenerateObservations(Observation &obs, double duration)
{
  BenchmarkData &data = mDetections;

  foreach_(
      BenchmarkData::Object &obj,
      data.mData.at(mTracker->mCurrentStep)) {
    obs.mDetections.push_back(
        make_shared<Detection>(
            obj.position,
            -1.0, //no orientation
            obj.confidence,
            obj.ih,
            obj.iw,
            obj.rh,
            obj.rw));
  }
}

void BenchmarkSimulator::Display()
{
  mSignals[0].Set();
  mSignals[1].Wait();
}

void BenchmarkSimulator::LogRCG()
{
  if (mTracker->mRCGLogger && mGroundTruths.mField) {
    QRectF &rect = *mGroundTruths.mField;

    for (int i = 0; i <= HumanTracker::LOG_ALL; ++i) {
      mTracker->mRCGLogger[i]->LogRectangular(
          rect.left(), rect.right(), rect.top(), rect.bottom(), Qt::black);
    }
  }
}

void BenchmarkSimulator::BenchmarkData::InitMetaData()
{
  double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;

  if (mName == "PETS-S2L1") { //crop as in http://research.milanton.de/data.html
    xmin = -14.0696;
    xmax = 4.9813;
    ymin = -14.274;
    ymax = 1.7335;

    mStartNumber = 0;
    mRawDataPath =
        "MOT-benchmarks/data/Crowd_PETS09/S2/L1/frame_";
    mFormat = ".jpg";
  }
  else if (mName == "TUD-stadtmitte") { //crop as in http://research.milanton.de/data.html
    xmin = -0.019;
    xmax = 12.939;
    ymin = -0.048;
    ymax = 10.053;

    mStartNumber = 7022;
    mRawDataPath =
        "MOT-benchmarks/data//tud_stadtmitte/DaMultiview-seq";
    mFormat = ".png";
  }
  else if (mName == "CMU") {
    mData.resize(1673);

    mStartNumber = 0;
    mRawDataPath = "MOT-benchmarks/data/CMU/frame_";
    mFormat = ".png";

    Params::ins().cropped = false;
  }

  mCenter.set((xmin + xmax) / 2.0, (ymin + ymax) / 2.0);

  if (Params::ins().cropped) {
    mField = make_shared<QRectF>(xmin, ymin, xmax - xmin, ymax - ymin);
  }
}

bool BenchmarkSimulator::BenchmarkData::fromXml(
    std::istream& is, Etiseo::CameraModel &camera)
{
  xmlDocPtr doc = xmlReadIO(UtilXml::ReadCallback,
        UtilXml::InputCloseCallback, &is,  NULL, NULL, 0);
  xmlNodePtr node = xmlDocGetRootElement(doc);

  if (node) {
    if (xmlStrcmp(node->name, XML_TAG_DATASET) == 0) {
      char *name = (char*)xmlGetProp(node, XML_TAG_NAME);
      if (name) {
        mName = name;
        xmlFree(name);

        InitMetaData();
      }

      xmlNodePtr frame = node->xmlChildrenNode;
      std::list<Object> frame_data;

      while (frame != NULL) {
        if (xmlStrcmp(frame->name, XML_TAG_FRAME) == 0) {
          int frame_id = -1;

          char *number = (char*)xmlGetProp(frame, XML_TAG_NUMBER);
          if (number) {
            frame_id = atoi(number);
            xmlFree(number);
          }
          else {
            assert(0);
          }

          xmlNodePtr objectlist = frame->xmlChildrenNode;

          if (xmlStrcmp(objectlist->name, XML_TAG_OBJECTLIST) == 0) {
            xmlNodePtr object = objectlist->xmlChildrenNode;

            while (object != NULL) {
              Object obj;

              if (xmlStrcmp(object->name, XML_TAG_OBJECT) == 0) {
                char *confidence = (char*)xmlGetProp(object, XML_TAG_CONFIDENCE);
                if (confidence) {
                  obj.confidence = atof(confidence);
                  xmlFree(confidence);
                }

                char *id = (char*)xmlGetProp(object, XML_TAG_ID);
                if (id) {
                  obj.id = atoi(id);
                  xmlFree(id);
                }
              }
              else {
                break;
              }

              xmlNodePtr box = object->xmlChildrenNode;

              if (xmlStrcmp(box->name, XML_TAG_BOX) == 0) {
                char *h = (char*)xmlGetProp(box, XML_TAG_H);
                if (h) {
                  obj.ih = atof(h);
                  xmlFree(h);
                }
                else {
                  assert(0);
                }

                char *w = (char*)xmlGetProp(box, XML_TAG_W);
                if (w) {
                  obj.iw = atof(w);
                  xmlFree(w);
                }
                else {
                  assert(0);
                }

                char *xc = (char*)xmlGetProp(box, XML_TAG_XC);
                if (xc) {
                  obj.xc = atof(xc);
                  xmlFree(xc);
                }
                else {
                  assert(0);
                }

                char *yc = (char*)xmlGetProp(box, XML_TAG_YC);
                if (yc) {
                  obj.yc = atof(yc);
                  xmlFree(yc);
                }
                else {
                  assert(0);
                }
              }
              else {
                assert(0);
              }

              obj.frame = frame_id;

              double xc = obj.xc;
              double yc = obj.yc + 0.5 * obj.ih;

              obj.position = camera.ImageToGround(xc, yc);

              vector2d bottomleft = camera.ImageToGround(xc - 0.5 * obj.iw, yc);
              vector2d bottomright = camera.ImageToGround(xc + 0.5 * obj.iw, yc);

              obj.rw = (bottomleft - bottomright).length();
              obj.rh = obj.rw  / obj.iw * obj.ih;

              if (InFieldOfView(obj.position)) {
                frame_data.push_back(obj);
              }

              object = object->next;
            }
          }
          else {
            break;
          }
        }
        else {
          break;
        }

        mData.push_back(frame_data);
        frame_data.clear();

        frame = frame->next;
      }
    }
    else {
      assert(0);
    }
  }

  return true;
}

