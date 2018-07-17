#include "tuw_rqt_ordermanager/item_robot.h"

namespace tuw_rqt_ordermanager
{
ItemRobot::ItemRobot() : QObject(), QGraphicsItem(), QListWidgetItem()
{
  radius_ = 2;
}

QRectF ItemRobot::boundingRect() const
{
  qreal penWidth = 1;
  float x = pose_.position.x;
  float y = pose_.position.y;
  float z = pose_.position.z;

  return QRectF(x - radius_ - penWidth / 2 - 25, y - radius_ - penWidth / 2 - 25, x + radius_ + penWidth / 2 + 25,
                y + radius_ + penWidth / 2 + 25);
}

void ItemRobot::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  float x = pose_.position.x;
  float y = pose_.position.y;
  float z = pose_.position.z;

  painter->setPen(Qt::NoPen);
  painter->setBrush(*(new QColor(0, 0, 0, 25)));
  QRectF rect(x - 25, y - 25, 50, 50);
  int angzero = 90;
  int span = 90 * 16;
  int angle = (angzero)*16 - span / 2;
  painter->drawPie(rect, angle, span);

  painter->setPen(Qt::SolidLine);
  painter->setBrush(*(new QColor(0, 255, 0, 255)));
  painter->drawEllipse(QPointF(x, y), radius_, radius_);
}

void ItemRobot::setRobotName(QString robot_name)
{
  robot_name_ = robot_name;
  setText(robot_name);
}

void ItemRobot::setRobotRadius(float radius)
{
  radius_ = radius;
}

QString ItemRobot::getRobotName()
{
  return robot_name_;
}

void ItemRobot::setPose(geometry_msgs::Pose pose)
{
  pose_ = pose;

  tf::Pose tfpose;
  tf::poseMsgToTF(pose_, tfpose);
  float yaw = tf::getYaw(tfpose.getRotation());

  setTransformOriginPoint(QPointF(pose_.position.x, pose_.position.y));
  setRotation(90 - yaw * 180 / M_PI);
}

}  // end namespace tuw_rqt_ordermanager
