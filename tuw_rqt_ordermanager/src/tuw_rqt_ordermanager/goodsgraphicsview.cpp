#include "tuw_rqt_ordermanager/goodsgraphicsview.h"
#include <QMouseEvent>
#include <QApplication>
#include <QScrollBar>
#include <qmath.h>

#include <QTimeLine>

namespace tuw_rqt_ordermanager
{
// https://stackoverflow.com/questions/19113532/qgraphicsview-zooming-in-and-out-under-mouse-position-using-mouse-wheel/50517097#50517097
GoodsGraphicsView::GoodsGraphicsView(QWidget* parent) : QGraphicsView(parent)
{
  setTransformationAnchor(QGraphicsView::NoAnchor);
  setResizeAnchor(QGraphicsView::NoAnchor);
  num_scheduled_scalings_ = 0;
}

void GoodsGraphicsView::wheelEvent(QWheelEvent* event)
{
  wheel_event_mouse_pos_ = event->pos();

  int numDegrees = event->delta() / 8;
  int numSteps = numDegrees / 15;  // see QWheelEvent documentation
  num_scheduled_scalings_ += numSteps;
  if (num_scheduled_scalings_ * numSteps <
      0)  // if user moved the wheel in another direction, we reset previously scheduled scalings
    num_scheduled_scalings_ = numSteps;

  QTimeLine* anim = new QTimeLine(350, this);
  anim->setUpdateInterval(20);

  connect(anim, SIGNAL(valueChanged(qreal)), SLOT(scalingTime(qreal)));
  connect(anim, SIGNAL(finished()), SLOT(animFinished()));
  anim->start();
  event->accept();
}

void GoodsGraphicsView::scalingTime(qreal x)
{
  QPointF oldPos = mapToScene(wheel_event_mouse_pos_);

  qreal factor = 1.0 + qreal(num_scheduled_scalings_) / 300.0;
  scale(factor, factor);

  QPointF newPos = mapToScene(wheel_event_mouse_pos_);
  QPointF delta = newPos - oldPos;
  this->translate(delta.x(), delta.y());
}

void GoodsGraphicsView::animFinished()
{
  if (num_scheduled_scalings_ > 0)
    num_scheduled_scalings_--;
  else
    num_scheduled_scalings_++;

  sender()->~QObject();
}

void GoodsGraphicsView::mousePressEvent(QMouseEvent* event)
{
  if (event->button() == Qt::LeftButton)
  {
    pan_ = false;
    float x = event->x();
    float y = event->y();
    float z = 0;
    QPointF scenePoint = mapToScene(x, y);
    x = scenePoint.x();
    y = scenePoint.y();
    emit goodAddPose(x, y, z);
    event->accept();
    return;
  }
  else if (event->button() == Qt::RightButton)
  {
    pan_ = true;
    pan_start_x_ = event->x();
    pan_start_y_ = event->y();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }
  event->ignore();
}
void GoodsGraphicsView::mouseReleaseEvent(QMouseEvent* event)
{
  if (event->button() == Qt::LeftButton)
  {
    event->ignore();
    return;
  }
  else if (event->button() == Qt::RightButton)
  {
    pan_ = false;
    setCursor(Qt::ArrowCursor);
    event->accept();
    return;
  }
  event->ignore();
}
void GoodsGraphicsView::mouseMoveEvent(QMouseEvent* event)
{
  if (pan_)
  {
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - (event->x() - pan_start_x_));
    verticalScrollBar()->setValue(verticalScrollBar()->value() - (event->y() - pan_start_y_));
    pan_start_x_ = event->x();
    pan_start_y_ = event->y();
    event->accept();
    return;
  }
  event->ignore();
}

}  // end namespace tuw_rqt_ordermanager
