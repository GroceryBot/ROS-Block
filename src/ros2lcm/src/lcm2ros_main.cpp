#include "Ros2lcm.h"

int main(int argc, char **argv)
{
    ROS_INFO("Started executable");

    ros::init(argc, argv, "lcm2ros");
    ros::NodeHandle n;
    //MULTICAST_URL
    lcm::LCM lcmInstance("udpm://239.255.76.67:7667?ttl=2");
    Ros2lcm r2lInstance(&lcmInstance, &n);
    lcmInstance.subscribe(SLAM_POSE_CHANNEL, &Ros2lcm::handle_slam_pose, &r2lInstance);

    ros::Publisher odom_pub = n.advertise<"nav_msgs::Odometry">("odom", 1000);
    // ROS_INFO("Pre spin");
    ros::Rate loop_rate(10);
    while (ros::ok())
    {   
        if(new_odom){
            odom_pub.publish(r2lInstance.get_odom());
            new_odom = false;
        }
        ros::spinOnce();
        lcmInstance.handleTimeout(1000);
        loop_rate.sleep();
    }

    return 0;
}
