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
#include <tuw_geometry_msgs/pose.h>
#include <tuw_multi_robot_msgs/Goods.h>
#include <tuw_multi_robot_msgs/GoodPosition.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>

namespace tuw_rqt_ordermanager
{

enum transform_ax {
    TRANSFORM_X,
    TRANSFORM_Y,
    TRANSFORM_Z,
    TRANSFORM_SCALAR
};

class Goods
    : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    Goods();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
        qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
        const qt_gui_cpp::Settings& instance_settings);

    // Comment in to signal that the plugin has a way to configure it
    // bool hasConfiguration() const;
    // void triggerConfiguration();

    void mapCallback(const nav_msgs::OccupancyGrid& msg);
    void odomCallback(const nav_msgs::Odometry& odom);
    void robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo& ri);
    void goodPoseCallback(const tuw_multi_robot_msgs::GoodPosition&);
    float transform_map_to_scene(int ax, float v);
    float transform_scene_to_map(int ax, float v);
    geometry_msgs::Pose transform_scene_to_map(geometry_msgs::Pose);
    geometry_msgs::Pose transform_map_to_scene(geometry_msgs::Pose);
private:
    Ui::GoodsWidget ui_;
    QWidget* widget_;
    QGraphicsScene scene;
    void subscribe_robot_odom();
    std::vector<ros::Subscriber> subscriptions;
    ros::Publisher pub_goods;

    float mapHeight;
    float mapOriginPositionX;
    float mapOriginPositionY;
    float mapOriginPositionZ;
    float mapResolution;

    std::vector<QColor> goodColors;
public slots:
    void set_map(const nav_msgs::OccupancyGrid&);
    void odom_handle(const nav_msgs::Odometry& odom);
    void robot_info_handle(const tuw_multi_robot_msgs::RobotInfo& ri);
    void good_position_handle(const tuw_multi_robot_msgs::GoodPosition&);

    void new_robot();
    void delete_robot();
    void edit_robot();
    void new_good();
    void delete_good();
    void edit_good();
    void send_goods();

    void good_add_pose(float x, float y, float z);
    void good_clear_poses();
signals:
    void map_changed(const nav_msgs::OccupancyGrid);
    void odom_received(const nav_msgs::Odometry);
    void robot_info_received(const tuw_multi_robot_msgs::RobotInfo& ri);
    void good_position_received(const tuw_multi_robot_msgs::GoodPosition&);
};

}  // namespace tuw_rqt_ordermanager
#endif  // TUW_RQT_ORDER_MANAGER_H
