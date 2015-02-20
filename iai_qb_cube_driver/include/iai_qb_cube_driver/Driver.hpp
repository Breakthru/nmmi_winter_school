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

    private:
      ros::NodeHandle nh_;
  };
}

#endif // IAI_QB_CUBE_DRIVER_DRIVER_HPP
