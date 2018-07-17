#ifndef TUW_RQT_ORDER_MANAGER_H
#define TUW_RQT_ORDER_MANAGER_H

#include <rqt_gui_cpp/plugin.h>
#include <tuw_rqt_ordermanager/goodsgraphicsview.h>
#include <tuw_rqt_ordermanager/ui_goods.h>
#include <tuw_rqt_ordermanager/robotdialog.h>
#include <tuw_rqt_ordermanager/gooddialog.h>
#include <tuw_rqt_ordermanager/item_robot.h>
#include <tuw_rqt_ordermanager/item_good.h>
#include <QWidget>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tuw_multi_robot_msgs/OrderArray.h>
#include <tuw_multi_robot_msgs/GoodPosition.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>

namespace tuw_rqt_ordermanager
{

enum TransformAxes
{
  TRANSFORM_X,
  TRANSFORM_Y,
  TRANSFORM_Z,
  TRANSFORM_SCALAR
};

class Goods : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  Goods();
  virtual void initPlugin(qt_gui_cpp::PluginContext&);
  virtual void shutdownPlugin();
  virtual void saveSettings(
      qt_gui_cpp::Settings& plugin_settings, 
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(
      const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

  void mapCallback(const nav_msgs::OccupancyGrid&);
  void odomCallback(const nav_msgs::Odometry&);
  void robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo&);
  void goodPoseCallback(const tuw_multi_robot_msgs::GoodPosition&);
  float transformMapToScene(int ax, float v);
  float transformSceneToMap(int ax, float v);
  geometry_msgs::Pose transformSceneToMap(geometry_msgs::Pose);
  geometry_msgs::Pose transformMapToScene(geometry_msgs::Pose);

private:
  void subscribeRobotOdom();

  Ui::GoodsWidget ui_;
  QWidget* widget_;
  QGraphicsScene scene_;
  std::vector<ros::Subscriber> subscriptions_;
  ros::Publisher pub_goods_;

  float map_height_;
  float map_origin_position_x_;
  float map_origin_position_y_;
  float map_origin_position_z_;
  float map_resolution_;

  std::vector<QColor> good_colors_;

public slots:
  void setMap(const nav_msgs::OccupancyGrid&);
  void odomHandle(const nav_msgs::Odometry&);
  void robotInfoHandle(const tuw_multi_robot_msgs::RobotInfo&);
  void goodPositionHandle(const tuw_multi_robot_msgs::GoodPosition&);

  void newRobot();
  void deleteRobot();
  void editRobot();
  void newGood();
  void deleteGood();
  void editGood();
  void sendGoods();

  void goodAddPose(float x, float y, float z);
  void goodClearPoses();

signals:
  void mapChanged(const nav_msgs::OccupancyGrid);
  void odomReceived(const nav_msgs::Odometry);
  void robotInfoReceived(const tuw_multi_robot_msgs::RobotInfo&);
  void goodPositionReceived(const tuw_multi_robot_msgs::GoodPosition&);
};

}  // namespace tuw_rqt_ordermanager
#endif  // TUW_RQT_ORDER_MANAGER_H
