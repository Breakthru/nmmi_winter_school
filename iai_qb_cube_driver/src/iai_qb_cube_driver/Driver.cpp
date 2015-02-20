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
  if(!readParameters())
    return;

  if(!startCommunication(port_))
    return;

  startPublisher(joint_names_);

  ros::Rate r(publish_frequency_);
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

// activation of cubes...
//  char tmp = 0x00
//  while(tmp == 0x00) {
//    // if (cube_id > 30) break; // Georg wonders why 30?
//    commActivate(&cube_comm_, cube_id, 1);
//    usleep(100000);
//    commGetActivate(&cube_comm_, cube_id, &tmp);
//    usleep(100000);
//  }
	
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

bool Driver::readParameters()
{
  if(!nh_.getParam("port", port_))
  {
    ROS_ERROR("No parameter 'port' in namespace %s found", 
        nh_.getNamespace().c_str());
    return false;
  }

  if(!nh_.getParam("publish_frequency", publish_frequency_))
  {
    ROS_ERROR("No parameter 'publish_frequency' in namespace %s found", 
        nh_.getNamespace().c_str());
    return false;
  }

  joint_names_.clear();
  if(!nh_.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("No parameter 'joint_names' in namespace %s found", 
        nh_.getNamespace().c_str());
    return false;
  }

  cube_id_map_.clear();
  for (size_t i=0; i<joint_names_.size(); i++)
  {
    int cube_id; 
    if(!nh_.getParam(joint_names_[i] + "/id", cube_id))
    {
      ROS_ERROR("No parameter '%s/id' in namespace %s found", 
          joint_names_[i].c_str(), nh_.getNamespace().c_str());
      return false;
    }
    cube_id_map_.insert(std::pair<std::string, int>(joint_names_[i], cube_id));
  }

  std::cout << "joint-names:\n";
  for (size_t i=0; i<joint_names_.size(); i++)
    std::cout << joint_names_[i] << "\n";

  std::cout << "joint-name -> cube-id:\n";
  for (std::map<std::string,int>::iterator it=cube_id_map_.begin();  
       it!=cube_id_map_.end(); ++it)
    std::cout << it->first << " => " << it->second << '\n';

  return true;
}
