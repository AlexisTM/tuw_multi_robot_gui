#ifndef TUW_RQT_ITEMGOOD_H
#define TUW_RQT_ITEMGOOD_H

#include <QObject>
#include <QGraphicsView>

#include <QGraphicsItem>
#include <QListWidgetItem>
#include <tuw_geometry_msgs/pose.h>

namespace tuw_rqt_ordermanager {

enum drawing_modes {
    DRAWING_MODE_PLAN,
    DRAWING_MODE_EXEC
};

class ItemGood : public QObject, public QGraphicsItem, public QListWidgetItem
{
    Q_OBJECT
public:
    explicit ItemGood();
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget);

    void setGoodName(QString);
    QString getGoodName();
    void setId(int);
    int getId();

    void addPose(tuw::ros_msgs::Pose*);
    void clearPoses();
    std::vector<tuw::ros_msgs::Pose*> getPoses();

    void setColor(QColor &);

    static const float ITEM_SIZE = 3;

    void setDrawingMode(int);
    void setCurrentPose(tuw::ros_msgs::Pose*);

private:
    int id;
    QString goodName;

    std::vector<tuw::ros_msgs::Pose*> poses;
    tuw::ros_msgs::Pose* currentPose;
    QBrush *coloredBrush;
    QPen *coloredPen;
    QColor color;

    int drawingMode;
};

} // namespace tuw_rqt_ordermanager

#endif


