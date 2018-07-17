#ifndef TUW_RQT_GOODSGRAPHICSVIEW_H
#define TUW_RQT_GOODSGRAPHICSVIEW_H

#include <QWidget>
#include <QGraphicsView>

namespace tuw_rqt_ordermanager
{
class GoodsGraphicsView : public QGraphicsView
{
  Q_OBJECT

public:
  explicit GoodsGraphicsView(QWidget* parent = 0);

signals:
  void goodAddPose(float x, float y, float z);

public slots:
  void wheelEvent(QWheelEvent* event);
  void scalingTime(qreal x);
  void animFinished();

private:
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);

  bool pan_;
  int pan_start_x_, pan_start_y_;

  qreal num_scheduled_scalings_;
  QPoint wheel_event_mouse_pos_;
};

}  // namespace tuw_rqt_ordermanager

#endif
