#include "Ros2lcm.h"
#include "mbot_channels.h"
#include <lcm/lcm-cpp.hpp>
#include <iostream>

//ROS MSG

//  Header header            # timestamp in the header is the acquisition time of
//                          # the first ray in the scan.
//                          #
//                          # in frame frame_id, angles are measured around
//                          # the positive Z axis (counterclockwise, if Z is up)
//                          # with zero angle being forward along the x axis
//
// float32 angle_min        # start angle of the scan [rad]
// float32 angle_max        # end angle of the scan [rad]
// float32 angle_increment  # angular distance between measurements [rad]
//
// float32 time_increment   # time between measurements [seconds] - if your scanner
//                          # is moving, this will be used in interpolating position
//                          # of 3d points
// float32 scan_time        # time between scans [seconds]
//
// float32 range_min        # minimum range value [m]
// float32 range_max        # maximum range value [m]
//
// float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
// float32[] intensities    # intensity data [device-specific units].  If your
//                          # device does not provide intensities, please leave
//                          # the array empty.
///////////////////////////////////////////////////////////
// struct lidar_t
// {
//     int64_t utime;
//
//     // Measured range in meters
//     // Measurement angle in radians
//     // Measurement time was taken in usec
//     // Measurement intensity -- unitless
//     int32_t num_ranges;
//     float   ranges[num_ranges];         // [m]
//     float   thetas[num_ranges];         // [rad]
//     int64_t times[num_ranges];          // [usec]
//     float   intensities[num_ranges];    // no units
// }

Ros2lcm::Ros2lcm(lcm::LCM * lcmInstance, ros::NodeHandle * nodeInstance){
    lcmInstance_ = lcmInstance;
    nodeInstance_ = nodeInstance;
    currentTime_ = 0;
}

void Ros2lcm::handle_timesync(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const timestamp_t* ts){
    //   currentTime_ = ts->utime;
      // ROS_INFO("Timesync info found");

}

void Ros2lcm::handle_odometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const odometry_t* odom){
    currentTime_ = odom->utime;

    //nodeInstance->publish
}


void Ros2lcm::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    //Subscribe to timesync
    ROS_INFO("STARTED SCAN CALL BACK");

    // ROS_INFO("Timesynce data found");
        ROS_INFO("%f",msg->angle_increment);
        ROS_INFO("%f",msg->angle_max);
        ROS_INFO("%f",msg->angle_min);

    lidar_t ls;
    ls.utime = currentTime_;
    // ROS_INFO("%ld", currentTime_);
    int sparser = 4;
    ls.num_ranges = (msg->angle_max - msg->angle_min)/msg->angle_increment;// + (-1*msg->angle_min)/msg->angle_increment/sparser;
    ROS_INFO("%d",ls.num_ranges);
    ls.ranges.resize(ls.num_ranges);
    ls.intensities.resize(ls.num_ranges);
    ls.thetas.resize(ls.num_ranges);
    ls.times.resize(ls.num_ranges);
    //   ROS_INFO("%d",ls.num_ranges);

    for(uint32_t i = 0; i < ls.num_ranges; i+=sparser){
            ROS_INFO("%d",i);
    if(isnan((float)msg->ranges[i])){
        ls.ranges[i/sparser] = 8.0;
    }
    else{
        ls.ranges[i/sparser] = std::min(msg->ranges[i],(float)8.0);
    }
        // ROS_INFO("%f",msg->ranges[i]);
      // ls.intensities[i] = msg->intensities[i];
    if(i*msg->angle_increment < msg->angle_max)
        ls.thetas[i/sparser] = msg->angle_max - i*msg->angle_increment;
    else
        ls.thetas[i/sparser] = -1 * (i*msg->angle_increment - msg->angle_max);

    // if(i == 0){
        ROS_INFO("%f",ls.thetas[i/sparser]);
                
    // }
    // //Do I need to add utime here
    ls.times[i/sparser] =  currentTime_ ;
    }
    lcmInstance_->publish(LIDAR_CHANNEL, &ls);
}
