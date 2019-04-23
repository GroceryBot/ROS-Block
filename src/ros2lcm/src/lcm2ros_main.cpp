#include "Ros2lcm.h"

int main(int argc, char **argv)
{
    ROS_INFO("Started executable");

    ros::init(argc, argv, "lcm2ros");
    ros::NodeHandle n;
    //MULTICAST_URL
    lcm::LCM lcmInstance("udpm://239.255.76.67:7667?ttl=2");
    tf::TransformBroadcaster bt;
    Ros2lcm r2lInstance(&lcmInstance, &n, &bt);
    lcmInstance.subscribe(SLAM_POSE_CHANNEL, &Ros2lcm::handle_slam_pose, &r2lInstance);
    // lcmInstance.subscribe(ODOMETRY_CHANNEL, &Ros2lcm::handle_odometry, &r2lInstance);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ROS_INFO("Pre spin");
    while (ros::ok())
    {
        ROS_INFO("check");

        if(r2lInstance.odom_status()){
            ROS_INFO("odom");

            odom_pub.publish(r2lInstance.get_odom());
            r2lInstance.odom_written();
        }
        ros::spinOnce();
        lcmInstance.handleTimeout(1000);
    }

    return 0;
}
