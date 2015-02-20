#include <iai_qb_cube_driver/Driver.hpp>

using namespace iai_qb_cube_driver;

Driver::Driver(const ros::NodeHandle& nh): nh_(nh), running_(false)
{

}

Driver::~Driver()
{

}

bool Driver::start()
{
  std::string port = "/dev/ttyUSB0";
  startCommunication(port); 
}

bool Driver::startCommunication(const std::string port)
{
  if(isRunning())
    return true;

  openRS485(&cube_comm_, port.c_str());

  if (cube_comm_.file_handle <= 0) {
    ROS_WARN("Panic, cube file handle was invalid");
    return false;
  } else {
    ROS_INFO("Started communication to QBCubes");
    running_ = true;
    return true;
  }
}
