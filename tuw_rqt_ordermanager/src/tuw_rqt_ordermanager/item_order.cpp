#include "tuw_rqt_ordermanager/item_order.h"

#include <limits>

namespace tuw_rqt_ordermanager
{
ItemOrder::ItemOrder() : QObject(), QGraphicsItem(), QListWidgetItem()
{
  color_ = Qt::blue;
  colored_brush_ = new QBrush(color_);
  colored_pen_ = new QPen(color_);
  colored_pen_->setCosmetic(true);
  drawing_mode_ = DRAWING_MODE_PLAN;
  drawBoundingRect_ = false;
}

QRectF ItemOrder::boundingRect() const
{
  int minx = std::numeric_limits<int>::max();
  int miny = std::numeric_limits<int>::max();
  int maxx = std::numeric_limits<int>::min();
  int maxy = std::numeric_limits<int>::min();

  if (stations_.size() == 0)
    return QRectF(0, 0, 0, 0);

  for (int i = 0; i < stations_.size(); ++i)
  {
    int _station_id = stations_.at(i);
    geometry_msgs::Pose pose =
      ((ItemStation*)(lst_stations_->item(_station_id)))->getPose();

    if (minx > pose.position.x)
      minx = pose.position.x;
    if (miny > pose.position.y)
      miny = pose.position.y;
    if (maxx < pose.position.x)
      maxx = pose.position.x;
    if (maxy < pose.position.y)
      maxy = pose.position.y;
  }

  return QRectF(
      minx - ITEM_SIZE/2,
      miny - ITEM_SIZE/2,
      maxx - minx + ITEM_SIZE,
      maxy - miny + ITEM_SIZE);
}

/*
 * paint this orders route from station to station
 */
void ItemOrder::paint(
    QPainter* painter,
    const QStyleOptionGraphicsItem* option,
    QWidget* widget)
{
  setTransformOriginPoint(QPointF(0, 0));
  painter->setBrush(*colored_brush_);
  painter->setPen(*colored_pen_);

  QPointF* last_point = NULL;

  for (int i = 0; i < stations_.size(); ++i)
  {
    int _station_id = stations_.at(i);
    geometry_msgs::Pose pose =
      ((ItemStation*)(lst_stations_->item(_station_id)))->getPose();
    if (last_point != NULL)
      painter->drawLine(
          QPointF(
            last_point->x(),
            last_point->y()
          ),
          QPointF(
            pose.position.x,
            pose.position.y
          ));

    last_point = new QPointF(pose.position.x, pose.position.y);
  }

  for (int i = 0; i < stations_.size(); ++i)
  {
    int _station_id = stations_.at(i);
    geometry_msgs::Pose pose =
      ((ItemStation*)(lst_stations_->item(_station_id)))->getPose();
    painter->setBrush(*colored_brush_);
    QRectF* rect = new QRectF(
        pose.position.x - ITEM_SIZE / 2,
        pose.position.y - ITEM_SIZE / 2,
        ITEM_SIZE,
        ITEM_SIZE);
    painter->drawRoundedRect(*rect, 0, 0);
    if (i == 0)
    {
      painter->setBrush(QBrush(QColor(0, 0, 0, 0)));
      painter->drawEllipse(QPointF(
            pose.position.x,
            pose.position.y
          ),
          ITEM_SIZE * 2,
          ITEM_SIZE * 2);
    }
  }

  if (drawing_mode_ == DRAWING_MODE_EXEC)
  {
    color_.setAlpha(255);
    QBrush* brush = new QBrush(color_);
    painter->setBrush(*brush);
    QRectF* rect = new QRectF(
        current_pose_->position.x - ITEM_SIZE / 2,
        current_pose_->position.y - ITEM_SIZE / 2,
        ITEM_SIZE,
        ITEM_SIZE);
    painter->drawRoundedRect(*rect, 0, 0);
  }

  // debug bounding rect:
  if (drawBoundingRect_)
  {
    painter->setBrush(*(new QColor(0, 255, 0, 0)));
    painter->drawRect(this->boundingRect());
  }
}

void ItemOrder::setOrderName(QString order_name)
{
  order_name_ = order_name;
  setText(order_name);
}

void ItemOrder::setId(int id)
{
  id_ = id;
}

int ItemOrder::getId()
{
  return id_;
}

QString ItemOrder::getOrderName()
{
  return order_name_;
}

void ItemOrder::addStation(int station_id)
{
  stations_.push_back(station_id);
}

void ItemOrder::clearStations()
{
  stations_.clear();
}

std::vector<int> ItemOrder::getStations()
{
  return stations_;
}

void ItemOrder::setColor(QColor& color)
{
  color_ = color;
  colored_brush_ = new QBrush(color_);
  colored_pen_ = new QPen(color_);
  colored_pen_->setCosmetic(true);
}

void ItemOrder::setDrawingMode(int drawing_mode)
{
  drawing_mode_ = drawing_mode;

  if (drawing_mode_ == DRAWING_MODE_ACTIVE)
    color_.setAlpha(255);
  else
    color_.setAlpha(50);

  colored_brush_ = new QBrush(color_);
  colored_pen_ = new QPen(color_);
  colored_pen_->setCosmetic(true);
}

void ItemOrder::setCurrentPose(geometry_msgs::Pose* pose)
{
  current_pose_ = pose;
}

void ItemOrder::setStationsList(QListWidget* lst_stations)
{
  lst_stations_ = lst_stations;
}

void ItemOrder::setDrawBoundingRect(bool drawBoundingRect)
{
  drawBoundingRect_ = drawBoundingRect;
}

}  // end namespace tuw_rqt_ordermanager
