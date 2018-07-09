#ifndef TUW_RQT_ITEMROBOT_H
#define TUW_RQT_ITEMROBOT_H

#include <QObject>
#include <QGraphicsView>

#include <QGraphicsItem>
#include <QListWidgetItem>

#include <tuw_geometry_msgs/pose.h>
#include <tf/tf.h>
#include <cmath>

namespace tuw_rqt_ordermanager {

class ItemRobot : public QObject, public QGraphicsItem, public QListWidgetItem
{
    Q_OBJECT
public:
    explicit ItemRobot();
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget);
    void setRobotName(QString);
    QString getRobotName();
    void setRobotRadius(float);

    void setPose(geometry_msgs::Pose pose);

private:
    QString robotName;
    geometry_msgs::Pose pose;
    float radius;
};

} // namespace tuw_rqt_ordermanager

#endif

