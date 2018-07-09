# include "tuw_rqt_ordermanager/item_good.h"

#include <limits>

namespace tuw_rqt_ordermanager
{

ItemGood::ItemGood()
  : QObject(), QGraphicsItem(), QListWidgetItem()
{
    color = Qt::blue;
    coloredBrush = new QBrush(color);
    coloredPen = new QPen(color);
    this->drawingMode = DRAWING_MODE_PLAN;
}


QRectF ItemGood::boundingRect() const
{
    return QRectF(0,0,600,600);

    //TODO: doesnt work when zooming
    
    qreal penWidth = 1;
    int minx = std::numeric_limits<int>::max();
    int miny = std::numeric_limits<int>::max();
    int maxx = std::numeric_limits<int>::min();
    int maxy = std::numeric_limits<int>::min();

    if ( poses.size() == 0 )
    {
        minx = 0;
        miny = 0;
        maxx = 0;
        maxy = 0;
    }

    for (int i=0; i<poses.size(); ++i)
    {
        tuw::ros_msgs::Pose* pose = poses.at(i);
        if ( minx > pose->position.x )
            minx = pose->position.x;
        if ( miny > pose->position.y )
            miny = pose->position.y;
        if ( maxx < pose->position.x )
            maxx = pose->position.x;
        if ( maxy < pose->position.y )
            maxy = pose->position.y;
    }

    return QRectF(minx - penWidth/2, miny - penWidth/2, maxx + ITEM_SIZE + penWidth/2, maxy + ITEM_SIZE + penWidth/2);
}

void ItemGood::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                      QWidget *widget)
{
    painter->setBrush(*coloredBrush);
    painter->setPen(*coloredPen);

    QPointF *last_point = NULL;

    for (int i=0; i<poses.size(); ++i)
    {
        tuw::ros_msgs::Pose* pose = poses.at(i);
        if (last_point != NULL)
            painter->drawLine(QPointF(last_point->x(), last_point->y()), QPointF(pose->position.x, pose->position.y));

        last_point = new QPointF(pose->position.x, pose->position.y);
    }

    for (int i=0; i<poses.size(); ++i)
    {
        painter->setBrush(*coloredBrush);
        tuw::ros_msgs::Pose* pose = poses.at(i);
        QRectF *rect = new QRectF(pose->position.x-ITEM_SIZE/2, pose->position.y-ITEM_SIZE/2, ITEM_SIZE, ITEM_SIZE);
        painter->drawRoundedRect(*rect, 0, 0);
        if ( i == 0 ) {
            painter->setBrush(QBrush(QColor(0,0,0,0)));
            painter->drawEllipse(QPointF(pose->position.x, pose->position.y), ITEM_SIZE*2, ITEM_SIZE*2);
        }
    }

    if ( drawingMode == DRAWING_MODE_EXEC )
    {
        color.setAlpha(255);
        QBrush *brush = new QBrush(color);
        painter->setBrush(*brush);
        QRectF *rect = new QRectF(currentPose->position.x-ITEM_SIZE/2, currentPose->position.y-ITEM_SIZE/2, ITEM_SIZE, ITEM_SIZE);
        painter->drawRoundedRect(*rect, 0, 0);
    }

}

void ItemGood::setGoodName(QString goodName)
{
    this->goodName = goodName;
    setText(goodName);
}

void ItemGood::setId(int id)
{
    this->id = id;
}
int ItemGood::getId()
{
    return id;
}
QString ItemGood::getGoodName()
{
    return goodName;
}

void ItemGood::addPose(tuw::ros_msgs::Pose* pose)
{
    poses.push_back(pose);
}

void ItemGood::clearPoses()
{
    poses.clear();
}

std::vector<tuw::ros_msgs::Pose*> ItemGood::getPoses()
{
    return poses;
}

void ItemGood::setColor(QColor & color)
{
    this->color = color;
    coloredBrush = new QBrush(color);
    coloredPen = new QPen(color);
}

void ItemGood::setDrawingMode(int drawingMode)
{
    this->drawingMode = drawingMode;
    if ( drawingMode == DRAWING_MODE_PLAN )
        color.setAlpha(255);
    else
        color.setAlpha(50);
    coloredBrush = new QBrush(color);
    coloredPen = new QPen(color);
}

void ItemGood::setCurrentPose(tuw::ros_msgs::Pose* pose)
{
    currentPose = pose;
}

} // end namespace tuw_rqt_ordermanager
