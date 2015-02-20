#include <iai_qb_cube_driver/Driver.hpp>

using namespace iai_qb_cube_driver;

Driver::Driver(const ros::NodeHandle& nh): nh_(nh), running_(false)
{

}

Driver::~Driver()
{
  stopCommunication();
}

void Driver::run()
{
  // TODO: get me from param server
  std::string port = "/dev/ttyUSB0";
  double frequency = 20.0;
  std::vector<std::string> joint_names;
  joint_names.push_back("joint0");
  joint_names.push_back("joint1");

  if(!startCommunication(port))
    return;

  startPublisher(joint_names);

  ros::Rate r(frequency);
  while(ros::ok())
  {
    readMeasurements(); 
    msg_.header.stamp = ros::Time::now();
    pub_.publish(msg_);
    r.sleep();
  }
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

  // TODO: activate all cubes
}

bool Driver::startPublisher(const std::vector<std::string>& joint_names)
{
  pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 1);

  msg_.name = joint_names;
  msg_.position.resize(joint_names.size());
}

void Driver::stopCommunication()
{
  if(isRunning())
    return;

  // TODO: deactivate all cubes?

  closeRS485(&cube_comm_);
}

void Driver::readMeasurements()
{
  // TODO: implement me, overwriting the content of msg_   
}
