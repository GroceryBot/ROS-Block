#include "Ros2lcm.h"

int main(int argc, char **argv)
{
  ROS_INFO("Started executable");

  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle n;
  //MULTICAST_URL
  lcm::LCM lcmInstance("udpm://239.255.76.67:7667?ttl=2");
  Ros2lcm r2lInstance(&lcmInstance, &n);
  lcmInstance.subscribe(MBOT_TIMESYNC_CHANNEL, &Ros2lcm::handle_timesync, &r2lInstance);
  lcmInstance.subscribe(ODOMETRY_CHANNEL, &Ros2lcm::handle_odometry, &r2lInstance);

  ros::Subscriber sub = n.subscribe("scan", 1000, &Ros2lcm::scanCallback, &r2lInstance);
  // ROS_INFO("Pre spin");

  // ros::spin();
  // ROS_INFO("Pre spin");
  while(true){
    // lcmInstance.handle();
    ros::spinOnce();
    lcmInstance.handleTimeout(50);

  }

  return 0;
}
