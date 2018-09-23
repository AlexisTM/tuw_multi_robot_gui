#ifndef TUW_RQT_ITEMORDER_H
#define TUW_RQT_ITEMORDER_H

#include <QObject>
#include <QGraphicsView>

#include <QGraphicsItem>
#include <QListWidgetItem>
#include <geometry_msgs/Pose.h>
#include <tuw_rqt_ordermanager/item_station.h>

namespace tuw_rqt_ordermanager
{
enum DrawingModes
{
  DRAWING_MODE_PLAN,
  DRAWING_MODE_ACTIVE,
  DRAWING_MODE_EXEC
};

class ItemOrder : public QObject, public QGraphicsItem, public QListWidgetItem
{
  Q_OBJECT

public:
  static const float ITEM_SIZE = 3;

  explicit ItemOrder();
  QRectF boundingRect() const;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);

  void setOrderName(QString);
  QString getOrderName();
  void setId(int);
  int getId();

  //void addPose(geometry_msgs::Pose*);
  //void clearPoses();
  //std::vector<geometry_msgs::Pose*> getPoses();
  void addStation(int station_id);
  void clearStations();
  std::vector<int> getStations();

  void setColor(QColor&);

  void setDrawingMode(int);
  void setCurrentPose(geometry_msgs::Pose*);

  void setStationsList(QListWidget* lst_stations);
  void setDrawBoundingRect(bool);

private:
  int id_;
  QString order_name_;

  //std::vector<geometry_msgs::Pose*> poses_;
  std::vector<int> stations_;
  geometry_msgs::Pose* current_pose_;
  QBrush* colored_brush_;
  QPen* colored_pen_;
  QColor color_;

  QListWidget* lst_stations_;

  int drawing_mode_;
  bool drawBoundingRect_;
};

}  // namespace tuw_rqt_ordermanager

#endif
