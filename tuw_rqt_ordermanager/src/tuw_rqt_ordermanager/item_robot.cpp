# include "tuw_rqt_ordermanager/item_robot.h"

namespace tuw_rqt_ordermanager
{

ItemRobot::ItemRobot()
  : QObject(), QGraphicsItem(), QListWidgetItem()
{
    radius = 2;
}


QRectF ItemRobot::boundingRect() const
{
    qreal penWidth = 1;
    float x = pose.position.x;
    float y = pose.position.y;
    float z = pose.position.z;

    return QRectF(x-radius - penWidth/2 - 25, y-radius - penWidth/2 - 25,
                  x+radius + penWidth/2 + 25, y+radius + penWidth/2 + 25);
}

void ItemRobot::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                      QWidget *widget)
{
    float x = pose.position.x;
    float y = pose.position.y;
    float z = pose.position.z;

    painter->setPen(Qt::NoPen);
    painter->setBrush(*(new QColor(0,0,0,25)));
    QRectF rect(x-25,y-25,50,50);
    int angzero = 90;
    int span = 90*16;
    int angle = (angzero)*16 - span/2;
    painter->drawPie(rect, angle, span);

    painter->setPen(Qt::SolidLine);
    painter->setBrush(*(new QColor(0,255,0,255)));
    painter->drawEllipse(QPointF(x,y), radius, radius);

}

void ItemRobot::setRobotName(QString robotName)
{
    this->robotName = robotName;
    setText(robotName);
}

void ItemRobot::setRobotRadius(float r)
{
    this->radius = r;
}

QString ItemRobot::getRobotName()
{
    return robotName;
}

void ItemRobot::setPose(geometry_msgs::Pose pose)
{
    this->pose = pose;

    tf::Pose tfpose;
    tf::poseMsgToTF(pose, tfpose);
    float yaw = tf::getYaw(tfpose.getRotation());

    setTransformOriginPoint(QPointF(pose.position.x, pose.position.y));
    setRotation(90 - yaw * 180 / M_PI);
}

} // end namespace tuw_rqt_ordermanager

