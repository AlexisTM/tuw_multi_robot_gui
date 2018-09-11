#ifndef TUW_RQT_ITEMSTATION_H
#define TUW_RQT_ITEMSTATION_H

#include <QObject>
#include <QGraphicsView>

#include <QGraphicsItem>
#include <QListWidgetItem>

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <cmath>

namespace tuw_rqt_ordermanager
{
class ItemStation : public QObject, public QGraphicsItem, public QListWidgetItem
{
  Q_OBJECT
public:
  explicit ItemStation();
  QRectF boundingRect() const;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
  void setStationName(QString);
  QString getStationName();
  void setPose(geometry_msgs::Pose);
  void setId(int);
  int getId();

private:
  QString station_name_;
  geometry_msgs::Pose pose_;
  float radius_;
  int id_;
};

}  // namespace tuw_rqt_ordermanager

#endif

