#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "iai_qb_cube_driver/Driver.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iai_qb_cube_driver");
  ros::NodeHandle n("~");
  iai_qb_cube_driver::Driver my_driver(n);
  my_driver.run();
  return 0;
}
