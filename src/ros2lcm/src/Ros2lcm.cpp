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
/////////////////////////////////////////////////////////////////////
// struct _odometry_t
// {
//     int64_t    utime;
//     float      x;
//     float      y;
//     float      theta;
//     float      fwd_velocity;
//     float      ang_velocity;
//     float      left_velocity;
//     float      right_velocity;
// };
////////////////////////////////////////////////////////////////////////////
//Odometry
//string child_frame_id
// geometry_msgs/PoseWithCovariance pose
    // geometry_msgs/Pose pose
    //     geometry_msgs/Point position
    //     geometry_msgs/Quaternion orientation
    // float64[36] covariance
// geometry_msgs/TwistWithCovariance twist
    // geometry_msgs/Twist twist
    //     geometry_msgs/Vector3 linear
    //         float64 x
    //         float64 y
    //         float64 z
    //     geometry_msgs/Vector3 angular
    //         float64 x
    //         float64 y
    //         float64 z
    // float64[36] covariance

//http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
Ros2lcm::Ros2lcm(lcm::LCM * lcmInstance, ros::NodeHandle * nodeInstance){
    lcmInstance_ = lcmInstance;
    nodeInstance_ = nodeInstance;
    odom_broadcaster_ = nullptr;
    currentTime_ = 0;
    lastTime_ = 0;
    x_ = 0;
    y_ = 0;
    th_ = 0;
    new_odom_ = false;
}

Ros2lcm::Ros2lcm(lcm::LCM * lcmInstance, ros::NodeHandle * nodeInstance, tf::TransformBroadcaster * odom_broadcaster){
    lcmInstance_ = lcmInstance;
    nodeInstance_ = nodeInstance;
    odom_broadcaster_ = odom_broadcaster;
    currentTime_ = 0;
    lastTime_ = 0;
    x_ = 0;
    y_ = 0;
    th_ = 0;
    new_odom_ = false;
}

nav_msgs::Odometry Ros2lcm::get_odom(){
    return odom_;
}

bool Ros2lcm::odom_status(){
    return new_odom_;
}

void Ros2lcm::odom_written(){
    //LOCKING??
    new_odom_ = false;
}


void Ros2lcm::handle_timesync(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const timestamp_t* ts){
    //   currentTime_ = ts->utime;
      // ROS_INFO("Timesync info found");

}

void Ros2lcm::handle_odometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const odometry_t* odom){
    currentTime_ = odom->utime;

    //nodeInstance->publish
}
void Ros2lcm::handle_slam_pose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* slam_pose){

    currentTime_ = slam_pose->utime;
    if(currentTime_ == lastTime_) return;
    //microsecond to seconds
    double dt = (currentTime_ - lastTime_);// / 100000;
    double dx = slam_pose->x - x_;
    double dy = slam_pose->y - y_;
    ROS_INFO("dt %f, dx %f, dy %f", dt,dx,dy);
    //DO I NEED TO WRAP?
    double dtheta = slam_pose->theta - th_;

    x_ = slam_pose->x;
    y_ = slam_pose->y;
    th_ = slam_pose->theta;
    ROS_INFO("x %f, y %f, theta %f", x_,y_,th_);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp.nsec = currentTime_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster_->sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    odom_.header.stamp.nsec = currentTime_;
    odom_.header.frame_id = "odom";

    //set the position
    odom_.pose.pose.position.x = x_;
    odom_.pose.pose.position.y = y_;
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation = odom_quat;

    //set the velocity
    odom_.child_frame_id = "base_link";
    odom_.twist.twist.linear.x = dx/dt ;
    odom_.twist.twist.linear.y = dy/dt;
    odom_.twist.twist.angular.z = dtheta/dt;

    // //publish the message
    // odom_pub.publish(odom);
    new_odom_ = true;
    lastTime_ = currentTime_;
    //nodeInstance->publish
}


void Ros2lcm::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    //Subscribe to timesync
    ROS_INFO("STARTED SCAN CALL BACK");
    if(currentTime_ == 0){
        return;
    }

    // ROS_INFO("Timesynce data found");
        ROS_INFO("%f",msg->angle_increment);
        ROS_INFO("%f",msg->angle_max);
        ROS_INFO("%f",msg->angle_min);

    lidar_t ls;
    ls.utime = currentTime_;
    // ROS_INFO("%ld", currentTime_);
    int sparser = 7;
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
        ls.ranges[i/sparser] = 5.0;
    }
    else{
        ls.ranges[i/sparser] = std::min(msg->ranges[i],(float)5.0);
    }
        // ROS_INFO("%f",msg->ranges[i]);
      // ls.intensities[i] = msg->intensities[i];
    if(i*msg->angle_increment < msg->angle_max)
        ls.thetas[i/sparser] = msg->angle_max - i*msg->angle_increment;
    else
        ls.thetas[i/sparser] = -1 * (i*msg->angle_increment - msg->angle_max);

    // if(i == 0){
        // ROS_INFO("%f",ls.thetas[i/sparser]);

    // }
    // //Do I need to add utime here
    ls.times[i/sparser] =  currentTime_ ;
    }
    lcmInstance_->publish(LIDAR_CHANNEL, &ls);
}
