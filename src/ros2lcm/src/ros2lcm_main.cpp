#include "Ros2lcm.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle n;
  //MULTICAST_URL
  lcm::LCM lcmInstance("udpm://239.255.76.67:7667?ttl=2");
  Ros2lcm r2lInstance(&lcmInstance, &n);

  ros::Subscriber sub = n.subscribe("scan", 1000, &Ros2lcm::scanCallback, &r2lInstance);

  ros::spin();

  return 0;
}
