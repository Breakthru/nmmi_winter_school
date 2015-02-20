#include <iai_qb_cube_driver/Driver.hpp>
#include <sensor_msgs/JointState.h>

using namespace iai_qb_cube_driver;

Driver::Driver(const ros::NodeHandle& nh): nh_(nh), running_(false)
{

}

Driver::~Driver()
{
  stopCommunication();
}

bool Driver::start()
{
  std::string port = "/dev/ttyUSB0";
  return startCommunication(port) && startPublisher();
}

bool Driver::startCommunication(const std::string& port)
{
  if(isRunning())
    return true;

  openRS485(&cube_comm_, port.c_str());

  if (cube_comm_.file_handle <= 0) {
    ROS_ERROR("Could not connect to QBCubes");
    return false;
  } else {
    ROS_INFO("Started communication to QBCubes");
    running_ = true;
    return true;
  }
}

bool Driver::startPublisher()
{
  pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 1);
}

void Driver::stopCommunication()
{
  if(isRunning())
    return;

  closeRS485(&cube_comm_);
}
