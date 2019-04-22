#include "ros/ros.h"
#include "mbot_channels.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "lcmtypes/lidar_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/timestamp_t.hpp"
#include "lcmtypes/odometry_t.hpp"


class Ros2lcm{
public:
    Ros2lcm(lcm::LCM * lcmInstance, ros::NodeHandle * nodeInstance);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void handle_timesync(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const timestamp_t* ts);
    void handle_odometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const odometry_t* odom);

private:


    lcm::LCM * lcmInstance_;
    ros::NodeHandle * nodeInstance_;
    tf::TransformBroadcaster odom_broadcaster_;
    int64_t currentTime_;
};
