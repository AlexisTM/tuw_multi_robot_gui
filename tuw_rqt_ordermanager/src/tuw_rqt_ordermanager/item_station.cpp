#include "tuw_rqt_ordermanager/item_station.h"

namespace tuw_rqt_ordermanager
{
ItemStation::ItemStation() : QObject(), QGraphicsItem(), QListWidgetItem()
{
  radius_ = 5;
}

QRectF ItemStation::boundingRect() const
{
  qreal penWidth = 1;
  float x = pose_.position.x;
  float y = pose_.position.y;
  float z = pose_.position.z;

  return QRectF(x - radius_ - penWidth / 2 - 25, y - radius_ - penWidth / 2 - 25, x + radius_ + penWidth / 2 + 25,
                y + radius_ + penWidth / 2 + 25);
}

void ItemStation::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  float x = pose_.position.x;
  float y = pose_.position.y;
  float z = pose_.position.z;

  painter->setPen(Qt::SolidLine);
  painter->setBrush(*(new QColor(0, 0, 255, 255)));
  painter->drawRect(QRectF(x, y, radius_, radius_));
}

void ItemStation::setStationName(QString station_name)
{
  station_name_ = station_name;
  setText(station_name);
}

QString ItemStation::getStationName()
{
  return station_name_;
}

void ItemStation::setPose(geometry_msgs::Pose pose)
{
  pose_ = pose;

  setTransformOriginPoint(QPointF(pose_.position.x, pose_.position.y));
}

int ItemStation::getId()
{
  return id_;
}

void ItemStation::setId(int id)
{
  id_ = id;
}

}  // end namespace tuw_rqt_ordermanager
