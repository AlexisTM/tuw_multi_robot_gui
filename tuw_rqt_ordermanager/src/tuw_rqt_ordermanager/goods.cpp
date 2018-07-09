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

Goods::Goods()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    setObjectName("Goods");

    /*
    QStringList known_colors = QColor::colorNames();
    for ( int i=0; i<known_colors.size(); ++i)
    {
        std::cout << known_colors[i].toStdString() << std::endl;
    }
    */

    goodColors.push_back( "blue" );
    goodColors.push_back( "red" );
    goodColors.push_back( "green" );
    goodColors.push_back( "orange" );
    goodColors.push_back( "brown" );
    goodColors.push_back( "yellow" );
    goodColors.push_back( "pink" );
    goodColors.push_back( "gold" );
}

float Goods::transform_map_to_scene(int ax, float v)
{
    if ( ax == TRANSFORM_X )
        return (v-mapOriginPositionX)/mapResolution;
    if ( ax == TRANSFORM_Y )
        return mapHeight-(v-mapOriginPositionY)/mapResolution;
    if ( ax == TRANSFORM_Z )
        return (v-mapOriginPositionZ)/mapResolution;
    return v/mapResolution;
}

float Goods::transform_scene_to_map(int ax, float v)
{
    if ( ax == TRANSFORM_X )
        return mapOriginPositionX + v * mapResolution;
    if ( ax == TRANSFORM_Y )
        return mapOriginPositionY - (v - mapHeight) * mapResolution;
    if ( ax == TRANSFORM_Z )
        return mapOriginPositionZ + v * mapResolution;
    return v*mapResolution;
}

geometry_msgs::Pose Goods::transform_map_to_scene(geometry_msgs::Pose pose)
{
    pose.position.x = transform_map_to_scene(TRANSFORM_X, pose.position.x);
    pose.position.y = transform_map_to_scene(TRANSFORM_Y, pose.position.y);
    pose.position.z = transform_map_to_scene(TRANSFORM_Z, pose.position.z);
    return pose;
}

geometry_msgs::Pose Goods::transform_scene_to_map(geometry_msgs::Pose pose)
{
    pose.position.x = transform_scene_to_map(TRANSFORM_X, pose.position.x);
    pose.position.y = transform_scene_to_map(TRANSFORM_Y, pose.position.y);
    pose.position.z = transform_scene_to_map(TRANSFORM_Z, pose.position.z);
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


    //qRegisterMetaType<std::vector<int8_t> >("std::vector<int8_t>");
    qRegisterMetaType<nav_msgs::OccupancyGrid>("nav_msgs::OccupancyGrid");
    qRegisterMetaType<nav_msgs::Odometry>("nav_msgs::Odometry");
    qRegisterMetaType<tuw_multi_robot_msgs::RobotInfo>("tuw_multi_robot_msgs::RobotInfo");
    qRegisterMetaType<tuw_multi_robot_msgs::GoodPosition>("tuw_multi_robot_msgs::GoodPosition");

    connect(this, &Goods::map_changed, this, &Goods::set_map);
    connect(this, &Goods::odom_received, this, &Goods::odom_handle);
    connect(this, &Goods::robot_info_received, this, &Goods::robot_info_handle);
    connect(this, &Goods::good_position_received, this, &Goods::good_position_handle);

    connect(ui_.btn_new_robot, SIGNAL(clicked()), this, SLOT(new_robot()));
    connect(ui_.btn_delete_robot, SIGNAL(clicked()), this, SLOT(delete_robot()));
    connect(ui_.btn_edit_robot, SIGNAL(clicked()), this, SLOT(edit_robot()));

    connect(ui_.btn_new_good, SIGNAL(clicked()), this, SLOT(new_good()));
    connect(ui_.btn_delete_good, SIGNAL(clicked()), this, SLOT(delete_good()));
    connect(ui_.btn_edit_good, SIGNAL(clicked()), this, SLOT(edit_good()));

    connect(ui_.btn_start, SIGNAL(clicked()), this, SLOT(send_goods()));
    connect(ui_.btn_clear_good_poses, SIGNAL(clicked()), this, SLOT(good_clear_poses()));

    connect(ui_.map_view, &GoodsGraphicsView::good_add_pose, this, &Goods::good_add_pose);

	ui_.map_view->setScene(&scene);

    pub_goods = getNodeHandle().advertise<tuw_multi_robot_msgs::Goods>("goods", 1);

    subscriptions.push_back(getNodeHandle().subscribe("/map", 0, &tuw_rqt_ordermanager::Goods::mapCallback, this));
    subscriptions.push_back(getNodeHandle().subscribe("/robot_info", 10, &tuw_rqt_ordermanager::Goods::robotInfoCallback, this));
    subscriptions.push_back(getNodeHandle().subscribe("/good_pose", 10, &tuw_rqt_ordermanager::Goods::goodPoseCallback, this));
    subscribe_robot_odom();
}

void Goods::shutdownPlugin()
{
    // unregister all publishers here
    // TODO: how?
}

void Goods::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
    // instance_settings.setValue(k, v)
}

void Goods::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
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

void Goods::mapCallback(const nav_msgs::OccupancyGrid& msg) {
    // use signal/slot to avoid concurrent scene updates
    emit map_changed(msg);
}

void Goods::set_map(const nav_msgs::OccupancyGrid& map)
{
    int width = map.info.width;
    int height = map.info.height;
    std::vector<int8_t> data = map.data;

    mapHeight = height;
    mapResolution = map.info.resolution;
    mapOriginPositionX = map.info.origin.position.x;
    mapOriginPositionY = map.info.origin.position.y;
    mapOriginPositionZ = map.info.origin.position.z;

    int col = 0;
    int row = height;

    QPixmap *pix = new QPixmap(width, height);
    QPainter *paint = new QPainter(pix);

    for(int i=0; i<data.size(); ++i)
    {
        int val = data[i];
        // val domain:
        // -1: unknown
        // 0-100: occupancy
        
        if ( val == -1 )  {
            paint->setPen(*(new QColor(255,255,255,255)));
            paint->drawPoint(col, row);
        } else {
            int g = (100-val)/100*255;
            paint->setPen(*(new QColor(g,g,g,255)));
            paint->drawPoint(col, row);
        }

        col += 1;
        if ( col == width ) {
            row -= 1;
            col = 0;
        }
    }

    scene.addPixmap(*pix);
}

void Goods::new_good()
{
    GoodDialog *dialog = new GoodDialog();
    // TODO: make sure id is unique
    int id = ui_.lst_goods->count();
    std::stringstream sstm;
    sstm << "good_" << id;
    std::string good_name = sstm.str();
    dialog->setGoodName(QString::fromStdString(good_name));
    int ret = dialog->exec();
    if ( ret == QDialog::Accepted ) {
        ItemGood *ir = new ItemGood();

        ir->setId(id);
        ir->setGoodName(dialog->getGoodName());
        ir->setColor(goodColors[id % goodColors.size()]);

        /*
        ir->setPositionX(transform_map_to_scene(TRANSFORM_X, dialog->getPositionX()));
        ir->setPositionY(transform_map_to_scene(TRANSFORM_Y, dialog->getPositionY()));
        ir->setPositionZ(transform_map_to_scene(TRANSFORM_Z, dialog->getPositionZ()));
        */

        ir->setZValue(2);

        scene.addItem(ir);
        ui_.lst_goods->addItem(ir);
        ui_.lst_goods->setCurrentItem(ir);

    }
}

void Goods::delete_good()
{
    QList <QListWidgetItem*>list = ui_.lst_goods->selectedItems();
    for (int i=0; i<list.size(); ++i) {
        ItemGood *ir = (ItemGood*)list[i];
        delete ir;
    }
}

void Goods::edit_good()
{
    QList <QListWidgetItem*>list = ui_.lst_goods->selectedItems();
    if (list.size() < 1)
        return;
    ItemGood *ir = (ItemGood*)list[0];
    GoodDialog *dialog = new GoodDialog();
    dialog->setGoodName(ir->getGoodName());

    /*
    dialog->setPositionX(transform_scene_to_map(TRANSFORM_X, ir->getPositionX()));
    dialog->setPositionY(transform_scene_to_map(TRANSFORM_Y, ir->getPositionY()));
    dialog->setPositionZ(transform_scene_to_map(TRANSFORM_Z, ir->getPositionZ()));
    */
    
    int ret = dialog->exec();
    if ( ret == QDialog::Accepted ) {
        ir->setGoodName(dialog->getGoodName());
        /*
        ir->setPositionX(transform_map_to_scene(TRANSFORM_X, dialog->getPositionX()));
        ir->setPositionY(transform_map_to_scene(TRANSFORM_Y, dialog->getPositionY()));
        ir->setPositionZ(transform_map_to_scene(TRANSFORM_Z, dialog->getPositionZ()));
        */
        scene.update();
    }
}

void Goods::new_robot()
{
    RobotDialog *dialog = new RobotDialog();
    int ret = dialog->exec();
    if ( ret == QDialog::Accepted ) {
        ItemRobot *ir = new ItemRobot();
        ir->setRobotName(dialog->getRobotName());
        /*
        ir->setPositionX(dialog->getPositionX());
        ir->setPositionY(dialog->getPositionY());
        ir->setPositionZ(dialog->getPositionZ());
        */

        scene.addItem(ir);
        ui_.lst_robots->addItem(ir);
        ui_.lst_robots->setCurrentItem(ir);

    }
}

void Goods::delete_robot()
{
    QList <QListWidgetItem*>list = ui_.lst_robots->selectedItems();
    for (int i=0; i<list.size(); ++i) {
        ItemRobot *ir = (ItemRobot*)list[i];
        delete ir;
    }
}

void Goods::edit_robot()
{
    QList <QListWidgetItem*>list = ui_.lst_robots->selectedItems();
    if (list.size() < 1)
        return;
    ItemRobot *ir = (ItemRobot*)list[0];
    RobotDialog *dialog = new RobotDialog();
    dialog->setRobotName(ir->getRobotName());
    /*
    dialog->setPositionX(transform_scene_to_map(TRANSFORM_X, ir->getPositionX()));
    dialog->setPositionY(transform_scene_to_map(TRANSFORM_Y, ir->getPositionY()));
    dialog->setPositionZ(transform_scene_to_map(TRANSFORM_Z, ir->getPositionZ()));
    */
    
    int ret = dialog->exec();
    if ( ret == QDialog::Accepted ) {
        ir->setRobotName(dialog->getRobotName());
        /*
        ir->setPositionX(transform_map_to_scene(TRANSFORM_X, dialog->getPositionX()));
        ir->setPositionY(transform_map_to_scene(TRANSFORM_Y, dialog->getPositionY()));
        ir->setPositionZ(transform_map_to_scene(TRANSFORM_Z, dialog->getPositionZ()));
        */

        scene.update();
    }
}

void Goods::subscribe_robot_odom()
{
    //TODO: scan periodically
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        //if(std::regex_match(info.name, std::regex("/robot_(0-9)+/odom")))
        if (info.name.find("/robot_") == 0 && 
            (info.name.substr(std::max(5, (int)info.name.length()-5)) == std::string("/odom")))
        {
            subscriptions.push_back(
                getNodeHandle().subscribe(info.name, 1, &tuw_rqt_ordermanager::Goods::odomCallback, this));
        }
    }
}

void Goods::goodPoseCallback(const tuw_multi_robot_msgs::GoodPosition& gp)
{
    emit good_position_received(gp);
}

void Goods::robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo& ri) {
    emit robot_info_received(ri);
}

void Goods::good_position_handle(const tuw_multi_robot_msgs::GoodPosition& gp)
{
    for( int i=0; i<ui_.lst_goods->count(); ++i )
    {
        ItemGood *ig = (ItemGood*)ui_.lst_goods->item(i);
        if (ig->getId() == gp.good_id) {

            float x = transform_map_to_scene(TRANSFORM_X, gp.position.position.x);
            float y = transform_map_to_scene(TRANSFORM_Y, gp.position.position.y);
            float z = transform_map_to_scene(TRANSFORM_Z, gp.position.position.z);
            tuw::ros_msgs::Pose *pose = new tuw::ros_msgs::Pose(x, y, z, 0, 0, 0);

            ig->setCurrentPose(pose);
            scene.update();
            break;
        }
    }
}

void Goods::robot_info_handle(const tuw_multi_robot_msgs::RobotInfo& ri) {
    std::string robot_name = ri.robot_name;
    QString q_robot_name = QString::fromStdString(robot_name);

    QList<QListWidgetItem*>matched_robots = ui_.lst_robots->findItems(q_robot_name, Qt::MatchExactly);
    if (matched_robots.size() == 1) {
        ItemRobot *ir = (ItemRobot*)matched_robots[0];
        ir->setRobotRadius(transform_map_to_scene(TRANSFORM_SCALAR, ri.shape_variables[0]));
        scene.update();
    }

}

void Goods::odomCallback(const nav_msgs::Odometry& odom) {
    emit odom_received(odom);
}

void Goods::odom_handle(const nav_msgs::Odometry& odom) {
    std::string robot_name = odom.header.frame_id.substr(1, odom.header.frame_id.find_last_of('/')-1);

    QString q_robot_name = QString::fromStdString(robot_name);

    QList<QListWidgetItem*>matched_robots = ui_.lst_robots->findItems(q_robot_name, Qt::MatchExactly);
    ItemRobot *ir;
    if (matched_robots.size() == 0) {
        ir = new ItemRobot();
        ir->setRobotName(q_robot_name);
        ir->setZValue(1);
        //float r = transform_map_to_scene(
        //ir->setRobotRadius(r);

        scene.addItem(ir);
        ui_.lst_robots->addItem(ir);
    } else {
        ir = (ItemRobot*)matched_robots[0];
    }

    /*
    ir->setPositionX(transform_map_to_scene(TRANSFORM_X, odom.pose.pose.position.x));
    ir->setPositionY(transform_map_to_scene(TRANSFORM_Y, odom.pose.pose.position.y));
    ir->setPositionZ(transform_map_to_scene(TRANSFORM_Z, odom.pose.pose.position.z));
    */

    ir->setPose(transform_map_to_scene(odom.pose.pose));

    scene.update();
}

void Goods::send_goods()
{
    tuw_multi_robot_msgs::Goods goods_msg;
    for ( int i=0; i<ui_.lst_goods->count(); ++i)
    {
        ItemGood *ir = (ItemGood*)ui_.lst_goods->item(i);
        ir->setDrawingMode(DRAWING_MODE_EXEC);
        std::vector<tuw::ros_msgs::Pose*> poses = ir->getPoses();

        // poses empty, no need to transport it anywhere
        if ( poses.size() <= 1 )
            continue;

        tuw_multi_robot_msgs::Good good_msg;

        good_msg.good_name = ir->getGoodName().toStdString();
        good_msg.good_id = ir->getId();

        // initial pose:
        /*
        float x = transform_scene_to_map(TRANSFORM_X, ir->getPositionX());
        float y = transform_scene_to_map(TRANSFORM_Y, ir->getPositionY());
        float z = transform_scene_to_map(TRANSFORM_Z, ir->getPositionZ());
        */
        //tuw::ros_msgs::Pose *pose = new tuw::ros_msgs::Pose(x, y, z, 0, 0, 0);
        //good_msg.positions.push_back(*pose);

        // target poses:
        for (int j=0; j<poses.size(); ++j)
        {
            tuw::ros_msgs::Pose *pose = poses.at(j);
            float x = transform_scene_to_map(TRANSFORM_X, pose->position.x);
            float y = transform_scene_to_map(TRANSFORM_Y, pose->position.y);
            float z = transform_scene_to_map(TRANSFORM_Z, pose->position.z);

            pose = new tuw::ros_msgs::Pose(x, y, z, 0, 0, 0);
            good_msg.positions.push_back(*pose);
        }

        goods_msg.goods.push_back(good_msg);

    }

    pub_goods.publish(goods_msg);
}

void Goods::good_add_pose(float x, float y, float z)
{
    QList <QListWidgetItem*>list = ui_.lst_goods->selectedItems();
    if (list.size() < 1)
    {
        return;
    }
    ItemGood *ir = (ItemGood*)list[0];

    tuw::ros_msgs::Pose *pose = new tuw::ros_msgs::Pose(x, y, z, 0, 0, 0);
    if ( ir->getPoses().size() == 0 )
        ir->setCurrentPose(pose);
    ir->addPose(pose);
    scene.update();
}

void Goods::good_clear_poses()
{
    QList <QListWidgetItem*>list = ui_.lst_goods->selectedItems();
    if (list.size() < 1)
    {
        return;
    }
    ItemGood *ir = (ItemGood*)list[0];
    ir->clearPoses();
    scene.update();
}

}  // namespace tuw_rqt_ordermanager
PLUGINLIB_DECLARE_CLASS(tuw_rqt_ordermanager, Goods, tuw_rqt_ordermanager::Goods, rqt_gui_cpp::Plugin)
