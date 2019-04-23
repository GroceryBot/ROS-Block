#include "ros/ros.h"
#include <lcm/lcm-cpp.hpp>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "lcmtypes/timestamp_t.hpp"
#include "lcmtypes/odometry_t.hpp"
#include "lcmtypes/lidar_t.hpp"
#include "lcmtypes/pose_xyt_t.hpp"
#include "mbot_channels.h"



class Ros2lcm{
public:
    Ros2lcm(lcm::LCM * lcmInstance, ros::NodeHandle * nodeInstance);
    Ros2lcm(lcm::LCM * lcmInstance, ros::NodeHandle * nodeInstance, tf::TransformBroadcaster * odom_broadcaster);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void handle_timesync(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const timestamp_t* ts);
    void handle_odometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const odometry_t* odom);
    void handle_slam_pose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* slam_pose);
    bool odom_status();
    void odom_written();
    //DO I NEED TO MAKE MULTITHREAD SAFE^^
    nav_msgs::Odometry get_odom();

private:
    lcm::LCM * lcmInstance_;
    ros::NodeHandle * nodeInstance_;
    tf::TransformBroadcaster * odom_broadcaster_;
    nav_msgs::Odometry odom_;
    int64_t currentTime_;
    int64_t lastTime_;
    double x_;
    double y_;
    double th_;
    bool new_odom_;

};
