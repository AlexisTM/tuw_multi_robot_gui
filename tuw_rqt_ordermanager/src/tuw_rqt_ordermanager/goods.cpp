/*
*/

#include <iostream>

#include "tuw_rqt_ordermanager/goods.h"
#include "tuw_rqt_ordermanager/goodsgraphicsview.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <QMetaType>
#include <QPixmap>
#include <QtGlobal>

#include <typeinfo>
#include <algorithm>
#include <string>
//#include <regex>

namespace tuw_rqt_ordermanager
{
Goods::Goods() : rqt_gui_cpp::Plugin(), widget_(0)
{
  setObjectName("Goods");

  /*
  QStringList known_colors = QColor::colorNames();
  for ( int i=0; i<known_colors.size(); ++i)
  {
      std::cout << known_colors[i].toStdString() << std::endl;
  }
  */

  good_colors_.push_back("blue");
  good_colors_.push_back("red");
  good_colors_.push_back("green");
  good_colors_.push_back("orange");
  good_colors_.push_back("brown");
  good_colors_.push_back("yellow");
  good_colors_.push_back("pink");
  good_colors_.push_back("gold");
}

float Goods::transformMapToScene(int ax, float v)
{
  if (ax == TRANSFORM_X)
    return (v - map_origin_position_x_) / map_resolution_;
  if (ax == TRANSFORM_Y)
    return map_height_ - (v - map_origin_position_y_) / map_resolution_;
  if (ax == TRANSFORM_Z)
    return (v - map_origin_position_z_) / map_resolution_;
  return v / map_resolution_;
}

float Goods::transformSceneToMap(int ax, float v)
{
  if (ax == TRANSFORM_X)
    return map_origin_position_x_ + v * map_resolution_;
  if (ax == TRANSFORM_Y)
    return map_origin_position_y_ - (v - map_height_) * map_resolution_;
  if (ax == TRANSFORM_Z)
    return map_origin_position_z_ + v * map_resolution_;
  return v * map_resolution_;
}

geometry_msgs::Pose Goods::transformMapToScene(geometry_msgs::Pose pose)
{
  pose.position.x = transformMapToScene(TRANSFORM_X, pose.position.x);
  pose.position.y = transformMapToScene(TRANSFORM_Y, pose.position.y);
  pose.position.z = transformMapToScene(TRANSFORM_Z, pose.position.z);
  return pose;
}

geometry_msgs::Pose Goods::transformSceneToMap(geometry_msgs::Pose pose)
{
  pose.position.x = transformSceneToMap(TRANSFORM_X, pose.position.x);
  pose.position.y = transformSceneToMap(TRANSFORM_Y, pose.position.y);
  pose.position.z = transformSceneToMap(TRANSFORM_Z, pose.position.z);
  return pose;
}

void Goods::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  // add widget to the user interface
  context.addWidget(widget_);

  qRegisterMetaType<nav_msgs::OccupancyGrid>("nav_msgs::OccupancyGrid");
  qRegisterMetaType<nav_msgs::Odometry>("nav_msgs::Odometry");
  qRegisterMetaType<tuw_multi_robot_msgs::RobotInfo>("tuw_multi_robot_msgs::RobotInfo");
  qRegisterMetaType<tuw_multi_robot_msgs::GoodPosition>("tuw_multi_robot_msgs::GoodPosition");

  connect(this, &Goods::mapChanged, this, &Goods::setMap);
  connect(this, &Goods::odomReceived, this, &Goods::odomHandle);
  connect(this, &Goods::robotInfoReceived, this, &Goods::robotInfoHandle);
  connect(this, &Goods::goodPositionReceived, this, &Goods::goodPositionHandle);

  connect(ui_.btn_new_robot, SIGNAL(clicked()), this, SLOT(newRobot()));
  connect(ui_.btn_delete_robot, SIGNAL(clicked()), this, SLOT(deleteRobot()));
  connect(ui_.btn_edit_robot, SIGNAL(clicked()), this, SLOT(editRobot()));

  connect(ui_.btn_new_good, SIGNAL(clicked()), this, SLOT(newGood()));
  connect(ui_.btn_delete_good, SIGNAL(clicked()), this, SLOT(deleteGood()));
  connect(ui_.btn_edit_good, SIGNAL(clicked()), this, SLOT(editGood()));

  connect(ui_.btn_start, SIGNAL(clicked()), this, SLOT(sendGoods()));
  connect(ui_.btn_clear_good_poses, SIGNAL(clicked()), this, SLOT(goodClearPoses()));

  connect(ui_.map_view, &GoodsGraphicsView::goodAddPose, this, &Goods::goodAddPose);

  ui_.map_view->setScene(&scene_);

  pub_goods_ = getNodeHandle().advertise<tuw_multi_robot_msgs::OrderArray>("/orders", 1);

  subscriptions_.push_back(
      getNodeHandle().subscribe("/map", 0, &tuw_rqt_ordermanager::Goods::mapCallback, this));
  subscriptions_.push_back(
      getNodeHandle().subscribe("/robot_info", 10, &tuw_rqt_ordermanager::Goods::robotInfoCallback, this));
  subscriptions_.push_back(
      getNodeHandle().subscribe("/good_position", 10, &tuw_rqt_ordermanager::Goods::goodPoseCallback, this));
  subscribeRobotOdom();
}

void Goods::shutdownPlugin()
{
  // unregister all publishers here
  // TODO: how?
}

void Goods::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void Goods::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
    return true;
}

void triggerConfiguration()
{
    // Usually used to open a dialog to offer the user a set of configuration
}*/

void Goods::mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  // use signal/slot to avoid concurrent scene updates
  emit mapChanged(msg);
}

void Goods::setMap(const nav_msgs::OccupancyGrid& map)
{
  int width = map.info.width;
  int height = map.info.height;
  std::vector<int8_t> data = map.data;

  map_height_ = height;
  map_resolution_ = map.info.resolution;
  map_origin_position_x_ = map.info.origin.position.x;
  map_origin_position_y_ = map.info.origin.position.y;
  map_origin_position_z_ = map.info.origin.position.z;

  int col = 0;
  int row = height;

  QPixmap* pix = new QPixmap(width, height);
  QPainter* paint = new QPainter(pix);

  for (int i = 0; i < data.size(); ++i)
  {
    int val = data[i];
    // val domain:
    // -1: unknown
    // 0-100: occupancy

    if (val == -1)
    {
      paint->setPen(*(new QColor(255, 255, 255, 255)));
      paint->drawPoint(col, row);
    }
    else
    {
      int g = (100 - val) / 100 * 255;
      paint->setPen(*(new QColor(g, g, g, 255)));
      paint->drawPoint(col, row);
    }

    col += 1;
    if (col == width)
    {
      row -= 1;
      col = 0;
    }
  }

  scene_.addPixmap(*pix);
}

void Goods::newGood()
{
  GoodDialog* dialog = new GoodDialog();
  // TODO: make sure id is unique
  int id = ui_.lst_goods->count();
  std::stringstream sstm;
  sstm << "good_" << id;
  std::string good_name = sstm.str();
  dialog->setGoodName(QString::fromStdString(good_name));
  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ItemGood* ir = new ItemGood();

    ir->setId(id);
    ir->setGoodName(dialog->getGoodName());
    ir->setColor(good_colors_[id % good_colors_.size()]);

    /*
    ir->setPositionX(transformMapToScene(TRANSFORM_X, dialog->getPositionX()));
    ir->setPositionY(transformMapToScene(TRANSFORM_Y, dialog->getPositionY()));
    ir->setPositionZ(transformMapToScene(TRANSFORM_Z, dialog->getPositionZ()));
    */

    ir->setZValue(2);

    scene_.addItem(ir);
    ui_.lst_goods->addItem(ir);
    ui_.lst_goods->setCurrentItem(ir);
  }
}

void Goods::deleteGood()
{
  QList<QListWidgetItem*> list = ui_.lst_goods->selectedItems();
  for (int i = 0; i < list.size(); ++i)
  {
    ItemGood* ir = (ItemGood*)list[i];
    delete ir;
  }
}

void Goods::editGood()
{
  QList<QListWidgetItem*> list = ui_.lst_goods->selectedItems();
  if (list.size() < 1)
    return;
  ItemGood* ir = (ItemGood*)list[0];
  GoodDialog* dialog = new GoodDialog();
  dialog->setGoodName(ir->getGoodName());

  /*
  dialog->setPositionX(transformSceneToMap(TRANSFORM_X, ir->getPositionX()));
  dialog->setPositionY(transformSceneToMap(TRANSFORM_Y, ir->getPositionY()));
  dialog->setPositionZ(transformSceneToMap(TRANSFORM_Z, ir->getPositionZ()));
  */

  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ir->setGoodName(dialog->getGoodName());
    /*
    ir->setPositionX(transformMapToScene(TRANSFORM_X, dialog->getPositionX()));
    ir->setPositionY(transformMapToScene(TRANSFORM_Y, dialog->getPositionY()));
    ir->setPositionZ(transformMapToScene(TRANSFORM_Z, dialog->getPositionZ()));
    */
    scene_.update();
  }
}

void Goods::newRobot()
{
  RobotDialog* dialog = new RobotDialog();
  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ItemRobot* ir = new ItemRobot();
    ir->setRobotName(dialog->getRobotName());
    /*
    ir->setPositionX(dialog->getPositionX());
    ir->setPositionY(dialog->getPositionY());
    ir->setPositionZ(dialog->getPositionZ());
    */

    scene_.addItem(ir);
    ui_.lst_robots->addItem(ir);
    ui_.lst_robots->setCurrentItem(ir);
  }
}

void Goods::deleteRobot()
{
  QList<QListWidgetItem*> list = ui_.lst_robots->selectedItems();
  for (int i = 0; i < list.size(); ++i)
  {
    ItemRobot* ir = (ItemRobot*)list[i];
    delete ir;
  }
}

void Goods::editRobot()
{
  QList<QListWidgetItem*> list = ui_.lst_robots->selectedItems();
  if (list.size() < 1)
    return;
  ItemRobot* ir = (ItemRobot*)list[0];
  RobotDialog* dialog = new RobotDialog();
  dialog->setRobotName(ir->getRobotName());
  /*
  dialog->setPositionX(transformSceneToMap(TRANSFORM_X, ir->getPositionX()));
  dialog->setPositionY(transformSceneToMap(TRANSFORM_Y, ir->getPositionY()));
  dialog->setPositionZ(transformSceneToMap(TRANSFORM_Z, ir->getPositionZ()));
  */

  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ir->setRobotName(dialog->getRobotName());
    /*
    ir->setPositionX(transformMapToScene(TRANSFORM_X, dialog->getPositionX()));
    ir->setPositionY(transformMapToScene(TRANSFORM_Y, dialog->getPositionY()));
    ir->setPositionZ(transformMapToScene(TRANSFORM_Z, dialog->getPositionZ()));
    */

    scene_.update();
  }
}

void Goods::subscribeRobotOdom()
{
  // TODO: use roboInfo instead of odom
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
  {
    const ros::master::TopicInfo& info = *it;
    // if(std::regex_match(info.name, std::regex("/robot_(0-9)+/odom")))
    if (info.name.find("/robot_") == 0 &&
        (info.name.substr(std::max(5, (int)info.name.length() - 5)) == std::string("/odom")))
    {
      subscriptions_.push_back(
          getNodeHandle().subscribe(info.name, 1, &tuw_rqt_ordermanager::Goods::odomCallback, this));
    }
  }
}

void Goods::goodPoseCallback(const tuw_multi_robot_msgs::GoodPosition& gp)
{
  emit goodPositionReceived(gp);
}

void Goods::robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo& ri)
{
  emit robotInfoReceived(ri);
}

void Goods::goodPositionHandle(const tuw_multi_robot_msgs::GoodPosition& gp)
{
  for (int i = 0; i < ui_.lst_goods->count(); ++i)
  {
    ItemGood* ig = (ItemGood*)ui_.lst_goods->item(i);
    if (ig->getId() == gp.good_id)
    {
      float x = transformMapToScene(TRANSFORM_X, gp.position.position.x);
      float y = transformMapToScene(TRANSFORM_Y, gp.position.position.y);
      float z = transformMapToScene(TRANSFORM_Z, gp.position.position.z);
      geometry_msgs::Pose* pose = new geometry_msgs::Pose();
      pose->position.x = x;
      pose->position.y = y;
      pose->position.z = z;

      ig->setCurrentPose(pose);
      scene_.update();
      break;
    }
  }
}

void Goods::robotInfoHandle(const tuw_multi_robot_msgs::RobotInfo& ri)
{
  std::string robot_name = ri.robot_name;
  QString q_robot_name = QString::fromStdString(robot_name);

  QList<QListWidgetItem*> matched_robots = ui_.lst_robots->findItems(q_robot_name, Qt::MatchExactly);
  if (matched_robots.size() == 1)
  {
    ItemRobot* ir = (ItemRobot*)matched_robots[0];
    ir->setRobotRadius(transformMapToScene(TRANSFORM_SCALAR, ri.shape_variables[0]));
    scene_.update();
  }
}

void Goods::odomCallback(const nav_msgs::Odometry& odom)
{
  emit odomReceived(odom);
}

void Goods::odomHandle(const nav_msgs::Odometry& odom)
{
  std::string robot_name = odom.header.frame_id.substr(1, odom.header.frame_id.find_last_of('/') - 1);

  QString q_robot_name = QString::fromStdString(robot_name);

  QList<QListWidgetItem*> matched_robots = ui_.lst_robots->findItems(q_robot_name, Qt::MatchExactly);
  ItemRobot* ir;
  if (matched_robots.size() == 0)
  {
    ir = new ItemRobot();
    ir->setRobotName(q_robot_name);
    ir->setZValue(1);

    scene_.addItem(ir);
    ui_.lst_robots->addItem(ir);
  }
  else
  {
    ir = (ItemRobot*)matched_robots[0];
  }

  ir->setPose(transformMapToScene(odom.pose.pose));

  scene_.update();
}

void Goods::sendGoods()
{
  tuw_multi_robot_msgs::OrderArray goods_msg;
  for (int i = 0; i < ui_.lst_goods->count(); ++i)
  {
    ItemGood* ir = (ItemGood*)ui_.lst_goods->item(i);
    ir->setDrawingMode(DRAWING_MODE_EXEC);
    std::vector<geometry_msgs::Pose*> poses = ir->getPoses();

    // poses empty, no need to transport it anywhere
    if (poses.size() <= 1)
      continue;

    tuw_multi_robot_msgs::Order good_msg;

    good_msg.good_name = ir->getGoodName().toStdString();
    good_msg.good_id = ir->getId();

    // target poses:
    for (int j = 0; j < poses.size(); ++j)
    {
      geometry_msgs::Pose* pose = poses.at(j);
      float x = transformSceneToMap(TRANSFORM_X, pose->position.x);
      float y = transformSceneToMap(TRANSFORM_Y, pose->position.y);
      float z = transformSceneToMap(TRANSFORM_Z, pose->position.z);

      // pose = new geometry_msgs::Pose(x, y, z, 0, 0, 0);
      geometry_msgs::Pose* new_pose = new geometry_msgs::Pose();
      new_pose->position.x = x;
      new_pose->position.y = y;
      new_pose->position.z = z;
      good_msg.positions.push_back(*new_pose);
    }

    goods_msg.orders.push_back(good_msg);
  }

  pub_goods_.publish(goods_msg);
}

void Goods::goodAddPose(float x, float y, float z)
{
  QList<QListWidgetItem*> list = ui_.lst_goods->selectedItems();
  if (list.size() < 1)
  {
    return;
  }
  ItemGood* ir = (ItemGood*)list[0];

  geometry_msgs::Pose* pose = new geometry_msgs::Pose();
  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = z;
  if (ir->getPoses().size() == 0)
    ir->setCurrentPose(pose);
  ir->addPose(pose);
  scene_.update();
}

void Goods::goodClearPoses()
{
  QList<QListWidgetItem*> list = ui_.lst_goods->selectedItems();
  if (list.size() < 1)
  {
    return;
  }
  ItemGood* ir = (ItemGood*)list[0];
  ir->clearPoses();
  scene_.update();
}

}  // end namespace tuw_rqt_ordermanager
PLUGINLIB_DECLARE_CLASS(tuw_rqt_ordermanager, Goods, tuw_rqt_ordermanager::Goods, rqt_gui_cpp::Plugin)
