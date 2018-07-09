#ifndef TUW_RQT_GOODSGRAPHICSVIEW_H
#define TUW_RQT_GOODSGRAPHICSVIEW_H

#include <QWidget>
#include <QGraphicsView>

namespace tuw_rqt_ordermanager {

class GoodsGraphicsView : public QGraphicsView
{
    Q_OBJECT

private:
	virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);

    bool _pan;
    int _panStartX, _panStartY;

    qreal _numScheduledScalings;
    QPoint wheelEventMousePos;
public:
    explicit GoodsGraphicsView(QWidget *parent = 0);

signals:
    void good_add_pose(float x, float y, float z);

public slots:
    void wheelEvent(QWheelEvent* event);
    void scalingTime(qreal x);
    void animFinished();
};

} // namespace tuw_rqt_ordermanager

#endif
