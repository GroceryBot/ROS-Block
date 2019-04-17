#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "lcmtypes/lidar_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/timestamp_t.hpp"

class Ros2lcm{
public:
    Ros2lcm(lcm::LCM * lcmInstance);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void handle_timesync(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const timestamp_t* ts);

private:


    lcm::LCM * lcmInstance_;
    uint32_t currentTime_;
};
