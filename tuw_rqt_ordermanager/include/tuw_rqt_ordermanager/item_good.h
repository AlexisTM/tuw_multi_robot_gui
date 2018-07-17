#ifndef TUW_RQT_ITEMGOOD_H
#define TUW_RQT_ITEMGOOD_H

#include <QObject>
#include <QGraphicsView>

#include <QGraphicsItem>
#include <QListWidgetItem>
#include <geometry_msgs/Pose.h>

namespace tuw_rqt_ordermanager
{
enum DrawingModes
{
  DRAWING_MODE_PLAN,
  DRAWING_MODE_EXEC
};

class ItemGood : public QObject, public QGraphicsItem, public QListWidgetItem
{
  Q_OBJECT

public:
  static const float ITEM_SIZE = 3;

  explicit ItemGood();
  QRectF boundingRect() const;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);

  void setGoodName(QString);
  QString getGoodName();
  void setId(int);
  int getId();

  void addPose(geometry_msgs::Pose*);
  void clearPoses();
  std::vector<geometry_msgs::Pose*> getPoses();

  void setColor(QColor&);

  void setDrawingMode(int);
  void setCurrentPose(geometry_msgs::Pose*);

private:
  int id_;
  QString good_name_;

  std::vector<geometry_msgs::Pose*> poses_;
  geometry_msgs::Pose* current_pose_;
  QBrush* colored_brush_;
  QPen* colored_pen_;
  QColor color_;

  int drawing_mode_;
};

}  // namespace tuw_rqt_ordermanager

#endif
