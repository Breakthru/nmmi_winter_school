#ifndef IAI_QB_CUBE_DRIVER_DRIVER_HPP
#define IAI_QB_CUBE_DRIVER_DRIVER_HPP

#include <qbmove_communications.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace iai_qb_cube_driver
{
  class Driver
  {
    public:
      Driver(const ros::NodeHandle& nh);
      ~Driver(); 

      void run();
  
      bool isRunning() const { return running_; }

    private:
      ros::NodeHandle nh_;
      comm_settings cube_comm_;
      ros::Publisher pub_;
      bool running_;
      sensor_msgs::JointState msg_;      

      bool startCommunication(const std::string& port);
      bool startPublisher(const std::vector<std::string>& joint_names);
      void stopCommunication();
      void readMeasurements();
  };
}

#endif // IAI_QB_CUBE_DRIVER_DRIVER_HPP
