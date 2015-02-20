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

      // TODO: make run threaded
      void run();
  
      bool isRunning() const { return running_; }

    private:
      // Communication stuff
      ros::NodeHandle nh_;
      comm_settings cube_comm_;
      ros::Publisher pub_;
      sensor_msgs::JointState msg_;      

      // Internal flags
      bool running_;

      // Internal data members
      std::vector<std::string> joint_names_;
      std::map<std::string, int> cube_id_map_;
      std::string port_;
      double publish_frequency_;

      bool startCommunication(const std::string& port);
      bool startPublisher(const std::vector<std::string>& joint_names);
      void stopCommunication();
      void readMeasurements();
      bool readParameters(); 
      // TODO: add a callback for commands
  };
}

#endif // IAI_QB_CUBE_DRIVER_DRIVER_HPP
