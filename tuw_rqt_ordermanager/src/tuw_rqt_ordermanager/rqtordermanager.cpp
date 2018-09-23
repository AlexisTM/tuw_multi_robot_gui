/*
*/

#include <iostream>

#include "tuw_rqt_ordermanager/rqtordermanager.h"
#include "tuw_rqt_ordermanager/graphicsview.h"
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
RQTOrdermanager::RQTOrdermanager() : rqt_gui_cpp::Plugin(), widget_(0)
{
  setObjectName("TUW RQT Ordermanager");

  order_colors_.push_back("blue");
  order_colors_.push_back("red");
  order_colors_.push_back("green");
  order_colors_.push_back("orange");
  order_colors_.push_back("brown");
  order_colors_.push_back("yellow");
  order_colors_.push_back("pink");
  order_colors_.push_back("gold");
}

void RQTOrdermanager::initPlugin(qt_gui_cpp::PluginContext& context)
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
  qRegisterMetaType<tuw_multi_robot_msgs::OrderPosition>("tuw_multi_robot_msgs::OrderPosition");
  qRegisterMetaType<tuw_multi_robot_msgs::StationArray>("tuw_multi_robot_msgs::StationArray");

  connect(this, &RQTOrdermanager::mapChanged, this, &RQTOrdermanager::setMap);
  connect(this, &RQTOrdermanager::odomReceived, this, &RQTOrdermanager::odomHandle);
  connect(this, &RQTOrdermanager::robotInfoReceived, this, &RQTOrdermanager::robotInfoHandle);
  connect(this, &RQTOrdermanager::orderPositionReceived, this, &RQTOrdermanager::orderPositionHandle);
  connect(this, &RQTOrdermanager::stationsReceived, this, &RQTOrdermanager::stationsHandle);

  connect(ui_.btn_new_robot, SIGNAL(clicked()), this, SLOT(newRobot()));
  connect(ui_.btn_delete_robot, SIGNAL(clicked()), this, SLOT(deleteRobot()));
  connect(ui_.btn_edit_robot, SIGNAL(clicked()), this, SLOT(editRobot()));

  connect(ui_.btn_new_station, SIGNAL(clicked()), this, SLOT(newStation()));
  connect(ui_.btn_delete_station, SIGNAL(clicked()), this, SLOT(deleteStation()));
  connect(ui_.btn_edit_station, SIGNAL(clicked()), this, SLOT(editStation()));

  connect(ui_.btn_new_order, SIGNAL(clicked()), this, SLOT(newOrder()));
  connect(ui_.btn_delete_order, SIGNAL(clicked()), this, SLOT(deleteOrder()));
  connect(ui_.btn_edit_order, SIGNAL(clicked()), this, SLOT(editOrder()));

  connect(ui_.btn_start, SIGNAL(clicked()), this, SLOT(sendOrders()));
  connect(ui_.btn_clear_order_poses, SIGNAL(clicked()), this, SLOT(orderClearPoses()));

  connect(ui_.map_view, &GraphicsView::orderAddStation, this, &RQTOrdermanager::orderAddStation);
  connect(ui_.map_view, &GraphicsView::newStation, this, &RQTOrdermanager::newStation);
  connect(ui_.map_view, &GraphicsView::removeStation, this, &RQTOrdermanager::deleteStationById);

  connect(ui_.lst_orders, &QListWidget::itemSelectionChanged, this, &RQTOrdermanager::ordersItemSelectionChanged);

  connect(ui_.cb_showBoundingRects, SIGNAL(clicked(bool)), this, SLOT(setDrawBoundingRects(bool)));

  ui_.map_view->setMapTransformation(&map_transformation_);
  ui_.map_view->setScene(&scene_);

  pub_orders_ = getNodeHandle().advertise<tuw_multi_robot_msgs::OrderArray>("/orders", 1);

  subscriptions_.push_back(
      getNodeHandle().subscribe("/map", 0, &tuw_rqt_ordermanager::RQTOrdermanager::mapCallback, this));
  subscriptions_.push_back(
      getNodeHandle().subscribe("/robot_info", 10, &tuw_rqt_ordermanager::RQTOrdermanager::robotInfoCallback, this));
  subscriptions_.push_back(
      getNodeHandle().subscribe("/order_position", 10, &tuw_rqt_ordermanager::RQTOrdermanager::orderPositionCallback, this));
  subscriptions_.push_back(
      getNodeHandle().subscribe("/stations", 10, &tuw_rqt_ordermanager::RQTOrdermanager::stationsCallback, this));
  subscribeRobotOdom();

}

void RQTOrdermanager::shutdownPlugin()
{
  getNodeHandle().shutdown();
}

void RQTOrdermanager::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void RQTOrdermanager::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
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

void RQTOrdermanager::mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  // use signal/slot to avoid concurrent scene updates
  emit mapChanged(msg);
}

void RQTOrdermanager::setMap(const nav_msgs::OccupancyGrid& map)
{
  int width = map.info.width;
  int height = map.info.height;
  std::vector<int8_t> data = map.data;

  map_transformation_.setMapHeight(height);
  map_transformation_.setMapResolution(map.info.resolution);
  map_transformation_.setMapOriginPositionX(map.info.origin.position.x);
  map_transformation_.setMapOriginPositionY(map.info.origin.position.y);
  map_transformation_.setMapOriginPositionZ(map.info.origin.position.z);

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

  // does not work in initPlugin, here it does (probably thread issue):
  this->requestUpdateOnce();
}

void RQTOrdermanager::newOrder()
{
  OrderDialog* dialog = new OrderDialog();
  // TODO: make sure id is unique
  int id = ui_.lst_orders->count();
  std::stringstream sstm;
  sstm << "order_" << id;
  std::string order_name = sstm.str();
  dialog->setOrderName(QString::fromStdString(order_name));
  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ItemOrder* ir = new ItemOrder();
    ir->setDrawBoundingRect(ui_.cb_showBoundingRects->isChecked());
    ir->setStationsList(ui_.lst_stations);

    ir->setId(id);
    ir->setOrderName(dialog->getOrderName());
    ir->setColor(order_colors_[id % order_colors_.size()]);

    /*
    ir->setPositionX(transformMapToScene(TRANSFORM_X, dialog->getPositionX()));
    ir->setPositionY(transformMapToScene(TRANSFORM_Y, dialog->getPositionY()));
    ir->setPositionZ(transformMapToScene(TRANSFORM_Z, dialog->getPositionZ()));
    */

    ir->setZValue(2);

    scene_.addItem(ir);
    ui_.lst_orders->addItem(ir);
    ui_.lst_orders->setCurrentItem(ir);
  }
}

void RQTOrdermanager::deleteOrder()
{
  QList<QListWidgetItem*> list = ui_.lst_orders->selectedItems();
  for (int i = 0; i < list.size(); ++i)
  {
    ItemOrder* ir = (ItemOrder*)list[i];
    delete ir;
  }
}

void RQTOrdermanager::editOrder()
{
  QList<QListWidgetItem*> list = ui_.lst_orders->selectedItems();
  if (list.size() < 1)
    return;
  ItemOrder* ir = (ItemOrder*)list[0];
  OrderDialog* dialog = new OrderDialog();
  dialog->setOrderName(ir->getOrderName());

  /*
  dialog->setPositionX(transformSceneToMap(TRANSFORM_X, ir->getPositionX()));
  dialog->setPositionY(transformSceneToMap(TRANSFORM_Y, ir->getPositionY()));
  dialog->setPositionZ(transformSceneToMap(TRANSFORM_Z, ir->getPositionZ()));
  */

  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ir->setOrderName(dialog->getOrderName());
    /*
    ir->setPositionX(transformMapToScene(TRANSFORM_X, dialog->getPositionX()));
    ir->setPositionY(transformMapToScene(TRANSFORM_Y, dialog->getPositionY()));
    ir->setPositionZ(transformMapToScene(TRANSFORM_Z, dialog->getPositionZ()));
    */
    scene_.update();
  }
}

void RQTOrdermanager::newRobot()
{
  RobotDialog* dialog = new RobotDialog();
  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {
    ItemRobot* ir = new ItemRobot();
    ir->setDrawBoundingRect(ui_.cb_showBoundingRects->isChecked());
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

void RQTOrdermanager::deleteRobot()
{
  QList<QListWidgetItem*> list = ui_.lst_robots->selectedItems();
  for (int i = 0; i < list.size(); ++i)
  {
    ItemRobot* ir = (ItemRobot*)list[i];
    delete ir;
  }
}

void RQTOrdermanager::setDrawBoundingRects(bool checked)
{
  for (int i = 0; i < ui_.lst_stations->count(); ++i)
  {
    ItemStation* is = (ItemStation*)ui_.lst_stations->item(i);
    is->setDrawBoundingRect(checked);
  }
  for (int i = 0; i < ui_.lst_robots->count(); ++i)
  {
    ItemRobot* is = (ItemRobot*)ui_.lst_robots->item(i);
    is->setDrawBoundingRect(checked);
  }
  for (int i = 0; i < ui_.lst_orders->count(); ++i)
  {
    ItemOrder* is = (ItemOrder*)ui_.lst_orders->item(i);
    is->setDrawBoundingRect(checked);
  }
}

void RQTOrdermanager::editRobot()
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

ItemStation* RQTOrdermanager::findStationById(int id)
{
  for (int i = 0; i < ui_.lst_stations->count(); ++i)
  {
    ItemStation* is = (ItemStation*)ui_.lst_stations->item(i);
    if ( is->getId() == id )
      return is;
  }

  return NULL;
}

int RQTOrdermanager::findUnusedStationId()
{
  //TODO: buggy. its not station id which must be unique but station_name
  int id = 0;
  for (int i = 0; i < ui_.lst_stations->count(); ++i)
  {
    ItemStation* is = (ItemStation*)ui_.lst_stations->item(i);
    //printf("found id %d\n", is->getId());
    if ( is->getId() >= id )
      id = is->getId() + 1;
  }

  return id;
}

void RQTOrdermanager::deleteStationById(int station_id)
{
  ItemStation* is = findStationById(station_id);
  tuw_multi_robot_srvs::StationManagerStationProtocol delStation;
  tuw_multi_robot_msgs::Station station;
  station.id = is->getId();
  station.name = is->getStationName().toStdString();

  delStation.request.request = "remove";
  delStation.request.station = station;
  if (ros::service::call("station_manager_station_service", delStation))
  {
    //TODO show message
  }
  else
  {
    //TODO show error
  }
}

void RQTOrdermanager::deleteStation()
{
  QList<QListWidgetItem*> list = ui_.lst_stations->selectedItems();
  for (int i = 0; i < list.size(); ++i)
  {
    deleteStationById(i);
  }
}

void RQTOrdermanager::newStation(float x, float y, float z)
{
  StationDialog* dialog = new StationDialog();
  dialog->setPositionX(x);
  dialog->setPositionY(y);
  dialog->setPositionZ(z);

  int proposed_id = findUnusedStationId();
  std::stringstream sstm;
  sstm << "station_" << proposed_id;
  std::string proposed_station_name = sstm.str();
  dialog->setStationName(QString::fromStdString(proposed_station_name));
  dialog->setStationId(proposed_id);
  
  int ret = dialog->exec();
  if (ret == QDialog::Accepted)
  {

  
    tuw_multi_robot_srvs::StationManagerStationProtocol addStation;
    tuw_multi_robot_msgs::Station station;
    station.id = dialog->getStationId(); //int32
    station.name = dialog->getStationName().toStdString(); //string

    geometry_msgs::Pose* pose = new geometry_msgs::Pose();
    pose->position.x = dialog->getPositionX();
    pose->position.y = dialog->getPositionY();
    pose->position.z = dialog->getPositionZ();
    station.pose = *pose;

    addStation.request.request = "append";
    addStation.request.station = station;
    if (ros::service::call("station_manager_station_service", addStation))
    {
      //TODO show message
    }
    else
    {
      //TODO show error
    }
  }
}

void RQTOrdermanager::subscribeRobotOdom()
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
          getNodeHandle().subscribe(info.name, 1, &tuw_rqt_ordermanager::RQTOrdermanager::odomCallback, this));
    }
  }
}

void RQTOrdermanager::orderPositionCallback(const tuw_multi_robot_msgs::OrderPosition& gp)
{
  emit orderPositionReceived(gp);
}

void RQTOrdermanager::stationsCallback(const tuw_multi_robot_msgs::StationArray& sa)
{
  emit stationsReceived(sa);
}

void RQTOrdermanager::robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo& ri)
{
  emit robotInfoReceived(ri);
}

void RQTOrdermanager::orderPositionHandle(const tuw_multi_robot_msgs::OrderPosition& gp)
{
  for (int i = 0; i < ui_.lst_orders->count(); ++i)
  {
    ItemOrder* ig = (ItemOrder*)ui_.lst_orders->item(i);
    if (ig->getId() == gp.order_id)
    {
      float x = map_transformation_.transformMapToScene(MapTransformation::TRANSFORM_X, gp.position.position.x);
      float y = map_transformation_.transformMapToScene(MapTransformation::TRANSFORM_Y, gp.position.position.y);
      float z = map_transformation_.transformMapToScene(MapTransformation::TRANSFORM_Z, gp.position.position.z);
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

void RQTOrdermanager::stationsHandle(const tuw_multi_robot_msgs::StationArray& sa)
{
  ui_.lst_stations->clear();
  for (int i = 0; i < sa.stations.size(); ++i)
  {
    tuw_multi_robot_msgs::Station station = sa.stations[i];

    // XXX: id from message seems to be not respected. use own id
    station.id = i;

    ItemStation* is = new ItemStation();
    is->setDrawBoundingRect(ui_.cb_showBoundingRects->isChecked());
    is->setId(station.id);
    is->setStationName(QString::fromStdString(station.name));
    is->setPose(map_transformation_.transformMapToScene(station.pose));

    scene_.addItem(is);
    connect(is, &ItemStation::setActiveStation, ui_.map_view, &GraphicsView::setActiveStation);
    ui_.lst_stations->addItem(is);
  }
}

void RQTOrdermanager::robotInfoHandle(const tuw_multi_robot_msgs::RobotInfo& ri)
{
  std::string robot_name = ri.robot_name;
  QString q_robot_name = QString::fromStdString(robot_name);

  QList<QListWidgetItem*> matched_robots = ui_.lst_robots->findItems(q_robot_name, Qt::MatchExactly);
  if (matched_robots.size() == 1)
  {
    ItemRobot* ir = (ItemRobot*)matched_robots[0];
    ir->setRobotRadius(map_transformation_.transformMapToScene(MapTransformation::TRANSFORM_SCALAR, ri.shape_variables[0]));
    scene_.update();
  }
}

void RQTOrdermanager::odomCallback(const nav_msgs::Odometry& odom)
{
  emit odomReceived(odom);
}

void RQTOrdermanager::odomHandle(const nav_msgs::Odometry& odom)
{
  std::string robot_name = odom.header.frame_id.substr(1, odom.header.frame_id.find_last_of('/') - 1);

  QString q_robot_name = QString::fromStdString(robot_name);

  QList<QListWidgetItem*> matched_robots = ui_.lst_robots->findItems(q_robot_name, Qt::MatchExactly);
  ItemRobot* ir;
  if (matched_robots.size() == 0)
  {
    ir = new ItemRobot();
    ir->setDrawBoundingRect(ui_.cb_showBoundingRects->isChecked());
    ir->setRobotName(q_robot_name);
    ir->setZValue(1);

    scene_.addItem(ir);
    ui_.lst_robots->addItem(ir);
  }
  else
  {
    ir = (ItemRobot*)matched_robots[0];
  }

  ir->setPose(map_transformation_.transformMapToScene(odom.pose.pose));

  scene_.update();
}

void RQTOrdermanager::sendOrders()
{
  tuw_multi_robot_msgs::OrderArray order_array_msg;
  for (int i = 0; i < ui_.lst_orders->count(); ++i)
  {
    ItemOrder* ir = (ItemOrder*)ui_.lst_orders->item(i);
    ir->setDrawingMode(DRAWING_MODE_EXEC);
    //std::vector<geometry_msgs::Pose*> poses = ir->getPoses();
    std::vector<int> station_ids = ir->getStations();

    // stations empty, no need to transport it anywhere
    if (station_ids.size() <= 1)
      continue;

    tuw_multi_robot_msgs::Order order_msg;

    order_msg.order_name = ir->getOrderName().toStdString();
    order_msg.order_id = ir->getId();

    // target stations:
    for (int j = 0; j < station_ids.size(); ++j)
    {
      int _station_id = station_ids[j];
      //TODO 
      //ItemStation* station = findStationById(_station_id);
      geometry_msgs::Pose pose = ((ItemStation*)(ui_.lst_stations->item(_station_id)))->getPose();
      //geometry_msgs::Pose* pose = poses.at(j);
      float x = map_transformation_.transformSceneToMap(MapTransformation::TRANSFORM_X, pose.position.x);
      float y = map_transformation_.transformSceneToMap(MapTransformation::TRANSFORM_Y, pose.position.y);
      float z = map_transformation_.transformSceneToMap(MapTransformation::TRANSFORM_Z, pose.position.z);

      // pose = new geometry_msgs::Pose(x, y, z, 0, 0, 0);
      geometry_msgs::Pose* new_pose = new geometry_msgs::Pose();
      new_pose->position.x = x;
      new_pose->position.y = y;
      new_pose->position.z = z;
      order_msg.positions.push_back(*new_pose);
    }

    order_array_msg.orders.push_back(order_msg);
  }

  pub_orders_.publish(order_array_msg);
}

//void RQTOrdermanager::orderAddPose(float x, float y, float z)
void RQTOrdermanager::orderAddStation(int station_id)
{
  QList<QListWidgetItem*> list = ui_.lst_orders->selectedItems();
  if (list.size() < 1)
  {
    return;
  }
  ItemOrder* ir = (ItemOrder*)list[0];

  /*
  geometry_msgs::Pose* pose = new geometry_msgs::Pose();
  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = z;
  if (ir->getPoses().size() == 0)
    ir->setCurrentPose(pose);
  ir->addPose(pose);
  */
  ir->addStation(station_id);
  scene_.update();
}

void RQTOrdermanager::orderClearPoses()
{
  QList<QListWidgetItem*> list = ui_.lst_orders->selectedItems();
  if (list.size() < 1)
  {
    return;
  }
  ItemOrder* ir = (ItemOrder*)list[0];
  ir->clearStations();
  scene_.update();
}

void RQTOrdermanager::requestUpdateOnce()
{
  tuw_multi_robot_srvs::StationManagerControlProtocol command;
  command.request.request = "update";
  command.request.addition = "once";
  if(ros::service::call("station_manager_control_service", command))
  {
    //TODO show message
  }
  else
  {
    //TODO show error
  }
}

void RQTOrdermanager::ordersItemSelectionChanged()
{
  for (int i = 0; i < ui_.lst_orders->count(); ++i)
  {
    ItemOrder* ir = (ItemOrder*)ui_.lst_orders->item(i);
    ir->setDrawingMode(DRAWING_MODE_PLAN);
  }

  QList<QListWidgetItem*> list = ui_.lst_orders->selectedItems();
  for (int i = 0; i < list.size(); ++i)
  {
    ItemOrder* ir = (ItemOrder*)list[i];
    ir->setDrawingMode(DRAWING_MODE_ACTIVE);
  }
}

}  // end namespace tuw_rqt_ordermanager
PLUGINLIB_DECLARE_CLASS(tuw_rqt_ordermanager, RQTOrdermanager, tuw_rqt_ordermanager::RQTOrdermanager, rqt_gui_cpp::Plugin)
