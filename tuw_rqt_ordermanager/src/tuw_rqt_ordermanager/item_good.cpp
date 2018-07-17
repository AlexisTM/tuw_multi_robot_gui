#include "tuw_rqt_ordermanager/item_good.h"

#include <limits>

namespace tuw_rqt_ordermanager
{
ItemGood::ItemGood() : QObject(), QGraphicsItem(), QListWidgetItem()
{
  color_ = Qt::blue;
  colored_brush_ = new QBrush(color_);
  colored_pen_ = new QPen(color_);
  drawing_mode_ = DRAWING_MODE_PLAN;
}

QRectF ItemGood::boundingRect() const
{
  return QRectF(0, 0, 600, 600);

  // TODO: doesnt work when zooming

  qreal penWidth = 1;
  int minx = std::numeric_limits<int>::max();
  int miny = std::numeric_limits<int>::max();
  int maxx = std::numeric_limits<int>::min();
  int maxy = std::numeric_limits<int>::min();

  if (poses_.size() == 0)
  {
    minx = 0;
    miny = 0;
    maxx = 0;
    maxy = 0;
  }

  for (int i = 0; i < poses_.size(); ++i)
  {
    geometry_msgs::Pose* pose = poses_.at(i);
    if (minx > pose->position.x)
      minx = pose->position.x;
    if (miny > pose->position.y)
      miny = pose->position.y;
    if (maxx < pose->position.x)
      maxx = pose->position.x;
    if (maxy < pose->position.y)
      maxy = pose->position.y;
  }

  return QRectF(minx - penWidth / 2, miny - penWidth / 2, maxx + ITEM_SIZE + penWidth / 2,
                maxy + ITEM_SIZE + penWidth / 2);
}

void ItemGood::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  painter->setBrush(*colored_brush_);
  painter->setPen(*colored_pen_);

  QPointF* last_point = NULL;

  for (int i = 0; i < poses_.size(); ++i)
  {
    geometry_msgs::Pose* pose = poses_.at(i);
    if (last_point != NULL)
      painter->drawLine(QPointF(last_point->x(), last_point->y()), QPointF(pose->position.x, pose->position.y));

    last_point = new QPointF(pose->position.x, pose->position.y);
  }

  for (int i = 0; i < poses_.size(); ++i)
  {
    painter->setBrush(*colored_brush_);
    geometry_msgs::Pose* pose = poses_.at(i);
    QRectF* rect = new QRectF(pose->position.x - ITEM_SIZE / 2, pose->position.y - ITEM_SIZE / 2, ITEM_SIZE, ITEM_SIZE);
    painter->drawRoundedRect(*rect, 0, 0);
    if (i == 0)
    {
      painter->setBrush(QBrush(QColor(0, 0, 0, 0)));
      painter->drawEllipse(QPointF(pose->position.x, pose->position.y), ITEM_SIZE * 2, ITEM_SIZE * 2);
    }
  }

  if (drawing_mode_ == DRAWING_MODE_EXEC)
  {
    color_.setAlpha(255);
    QBrush* brush = new QBrush(color_);
    painter->setBrush(*brush);
    QRectF* rect = new QRectF(current_pose_->position.x - ITEM_SIZE / 2, current_pose_->position.y - ITEM_SIZE / 2,
                              ITEM_SIZE, ITEM_SIZE);
    painter->drawRoundedRect(*rect, 0, 0);
  }
}

void ItemGood::setGoodName(QString good_name)
{
  good_name_ = good_name;
  setText(good_name);
}

void ItemGood::setId(int id)
{
  id_ = id;
}
int ItemGood::getId()
{
  return id_;
}
QString ItemGood::getGoodName()
{
  return good_name_;
}

void ItemGood::addPose(geometry_msgs::Pose* pose)
{
  poses_.push_back(pose);
}

void ItemGood::clearPoses()
{
  poses_.clear();
}

std::vector<geometry_msgs::Pose*> ItemGood::getPoses()
{
  return poses_;
}

void ItemGood::setColor(QColor& color)
{
  color_ = color;
  colored_brush_ = new QBrush(color_);
  colored_pen_ = new QPen(color_);
}

void ItemGood::setDrawingMode(int drawing_mode)
{
  drawing_mode_ = drawing_mode;
  if (drawing_mode_ == DRAWING_MODE_PLAN)
    color_.setAlpha(255);
  else
    color_.setAlpha(50);
  colored_brush_ = new QBrush(color_);
  colored_pen_ = new QPen(color_);
}

void ItemGood::setCurrentPose(geometry_msgs::Pose* pose)
{
  current_pose_ = pose;
}

}  // end namespace tuw_rqt_ordermanager
