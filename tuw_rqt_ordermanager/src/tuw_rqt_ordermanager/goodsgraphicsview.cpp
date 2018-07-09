#include "tuw_rqt_ordermanager/goodsgraphicsview.h"
#include <QMouseEvent>
#include <QApplication>
#include <QScrollBar>
#include <qmath.h>

#include <QTimeLine>

namespace tuw_rqt_ordermanager {

// https://stackoverflow.com/questions/19113532/qgraphicsview-zooming-in-and-out-under-mouse-position-using-mouse-wheel/50517097#50517097
GoodsGraphicsView::GoodsGraphicsView(QWidget *parent) : QGraphicsView(parent)
{
    setTransformationAnchor(QGraphicsView::NoAnchor);
    setResizeAnchor(QGraphicsView::NoAnchor);
    _numScheduledScalings = 0;
}

void GoodsGraphicsView::wheelEvent(QWheelEvent* event)
{
    wheelEventMousePos = event->pos();

    int numDegrees = event->delta() / 8;
    int numSteps = numDegrees / 15; // see QWheelEvent documentation
    _numScheduledScalings += numSteps;
    if (_numScheduledScalings * numSteps < 0) // if user moved the wheel in another direction, we reset previously scheduled scalings
        _numScheduledScalings = numSteps;

    QTimeLine *anim = new QTimeLine(350, this);
    anim->setUpdateInterval(20);

    connect(anim, SIGNAL (valueChanged(qreal)), SLOT (scalingTime(qreal)));
    connect(anim, SIGNAL (finished()), SLOT (animFinished()));
    anim->start();
    event->accept();
 }

void GoodsGraphicsView::scalingTime(qreal x)
{
    QPointF oldPos = mapToScene(wheelEventMousePos);

    qreal factor = 1.0+ qreal(_numScheduledScalings) / 300.0;
    scale(factor, factor);

    QPointF newPos = mapToScene(wheelEventMousePos);
    QPointF delta = newPos - oldPos;
    this->translate(delta.x(), delta.y());
}

void GoodsGraphicsView::animFinished()
{
    if (_numScheduledScalings > 0)
        _numScheduledScalings--;
    else
        _numScheduledScalings++;

    sender()->~QObject();
}

void GoodsGraphicsView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        _pan = false;
        float x = event->x();
        float y = event->y();
        float z = 0;
        QPointF scenePoint = mapToScene(x,y);
        x = scenePoint.x();
        y = scenePoint.y();
        emit good_add_pose(x, y, z);
        event->accept();
        return;
    } else if (event->button() == Qt::RightButton)
    {
        _pan = true;
        _panStartX = event->x();
        _panStartY = event->y();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    event->ignore();
}
void GoodsGraphicsView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        event->ignore();
        return;
    } else if (event->button() == Qt::RightButton)
    {
        _pan = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
        return;
    }
    event->ignore();
}
void GoodsGraphicsView::mouseMoveEvent(QMouseEvent *event)
{
    if (_pan)
    {
        horizontalScrollBar()->setValue(horizontalScrollBar()->value() - (event->x() - _panStartX));
        verticalScrollBar()->setValue(verticalScrollBar()->value() - (event->y() - _panStartY));
        _panStartX = event->x();
        _panStartY = event->y();
        event->accept();
        return;
    }
    event->ignore();

}

} // namespace tuw_rqt_ordermanager
