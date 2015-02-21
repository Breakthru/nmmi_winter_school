#ifndef IAI_QB_CUBE_DRIVER_DRIVER_HPP
#define IAI_QB_CUBE_DRIVER_DRIVER_HPP

#include <qbmove_communications.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iai_qb_cube_msgs/CubeCmd.h>

#include <pthread.h>


namespace iai_qb_cube_driver
{
  class Driver
  {
    public:
      Driver(const ros::NodeHandle& nh);
      ~Driver(); 

      // TODO: make run threaded
      void run();
  
      bool isCommunicationUp() const { return comm_up_; }
      bool areCubesActive() const { return cubes_active_; }


    private:
      // Communication stuff
      ros::NodeHandle nh_;
      comm_settings cube_comm_;
      ros::Publisher pub_;
      ros::Subscriber cmd_sub_;
      sensor_msgs::JointState msg_;
      int setPosStiff(short int pos, short int stiff, short int cube_id);


      void cmd_sub_cb_(const iai_qb_cube_msgs::CubeCmd::ConstPtr& msg);

      // Internal flags
      bool comm_up_, cubes_active_;

      // Internal data members
      std::map<std::string, int> cube_id_map_;
      std::string port_;
      double publish_frequency_;

      bool startCubeCommunication();
      void stopCubeCommunication();

      bool activateCubes();
      void deactivateCubes();

      bool readParameters(); 
      void initDatastructures();
      // TODO: add a callback for commands

      void readMeasurements();

      //Variables containing the commands that go out to the joints
      std::vector<float> des_joint_eqpoints;
      std::vector<float> des_joint_stiffness;


      //Things for the realtime thread
      double timeout_;
      bool exitRequested_;

      //members to stop and start the rt thread
      bool start_rt_thread(double timeout);
      void stop_rt_thread();

      //actual function running in the rt thread
      void* rt_run();

      //helper function to get the right pointer
      static void* run_s(void *ptr) { return ((Driver *) ptr)->rt_run(); }

      pthread_t thread_;
      pthread_mutex_t mutex_;

      //TODO: Need here variables for in-state and out-state



  };
}

#endif // IAI_QB_CUBE_DRIVER_DRIVER_HPP
