#ifndef IAI_QB_CUBE_DRIVER_DRIVER_HPP
#define IAI_QB_CUBE_DRIVER_DRIVER_HPP

#include <qbmove_communications.h>
#include <ros/ros.h>

namespace iai_qb_cube_driver
{
  class Driver
  {
    public:
      Driver(const ros::NodeHandle& nh);
      ~Driver(); 

      bool start();
  
      bool isRunning() const { return running_; }

    private:
      ros::NodeHandle nh_;
      comm_settings cube_comm_;
      bool running_;

      bool startCommunication(const std::string port);
      void stopCommunication();
  };
}

#endif // IAI_QB_CUBE_DRIVER_DRIVER_HPP
