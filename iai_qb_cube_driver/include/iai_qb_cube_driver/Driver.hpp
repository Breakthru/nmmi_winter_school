#ifndef IAI_QB_CUBE_DRIVER_DRIVER_HPP
#define IAI_QB_CUBE_DRIVER_DRIVER_HPP

#include <qbmove_communications.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iai_qb_cube_msgs/CubeCmdArray.h>
#include <iai_qb_cube_msgs/CubeStateArray.h>

#include <pthread.h>


namespace iai_qb_cube_driver
{
  class MotorCommand
  {
    public:
      short int motor1_position_;
      short int motor2_position_;
  };

  class InternalCommand
  {
    public:
      short int cube_id_;
      double equilibrium_point_;
      double stiffness_preset_;

      MotorCommand toMotorCommand() const;
  };

  class InternalState
  {
    public:
      short int cube_id_;
      double joint_position_;
      double motor1_position_;
      double motor2_position_; 
  };

  class Driver
  {
    public:
      Driver(const ros::NodeHandle& nh);
      ~Driver(); 

      // TODO: make run threaded
      void run();
  

    private:
      //
      // Communication stuff
      //
      ros::NodeHandle nh_;
      comm_settings cube_comm_;
      ros::Publisher pub_, cube_pub_;
      ros::Subscriber cmd_sub_;

      void cmd_sub_cb_(const iai_qb_cube_msgs::CubeCmdArray::ConstPtr& msg);
      sensor_msgs::JointState getJointStateMsg();
      iai_qb_cube_msgs::CubeStateArray getCubeStateMsg();

      int comm_error_counter_;

      //
      // Internal flags
      //
      bool comm_up_, cubes_active_, sim_mode_;

      bool isCommunicationUp() const { return comm_up_; }
      bool areCubesActive() const { return cubes_active_; }
      bool simulationMode() const { return sim_mode_; }

      //
      // Configuration of the node
      //
      std::map<std::string, int> cube_id_map_;
      std::string port_;
      double publish_frequency_;

      //
      // start and stop of cubes
      //
      bool startCubeCommunication();
      void stopCubeCommunication();

      bool activateCubes();
      void deactivateCubes();

      //
      // I/O with cubes
      //
      bool readParameters(); 
      void initDatastructures();

      void readMeasurement();
      void writeCommand();
      void commandSingleCube(const InternalCommand& command);

      //
      // Internal buffering between rt-thread and non-rt-thread
      //
      std::vector<InternalCommand> command_buffer_;
      std::vector<InternalState> measurement_buffer_, measurement_tmp_;

      const std::vector<InternalCommand>& readCommandBuffer();
      void writeCommandBuffer(const std::vector<InternalCommand>& new_command);
      const std::vector<InternalState>& readMeasurementBuffer();
      void writeMeasurementBuffer(const std::vector<InternalState>& new_measurement);

      //
      // rt-thread infrastructure
      //
      bool start_rt_thread(double timeout);
      void stop_rt_thread();

      //actual function running in the rt thread
      void* rt_run();

      //helper function to get the right pointer
      static void* run_s(void *ptr) { return ((Driver *) ptr)->rt_run(); }

      //Things for the realtime thread
      double timeout_;
      bool exitRequested_;
      pthread_t thread_;
      pthread_mutex_t mutex_;
  };
}

#endif // IAI_QB_CUBE_DRIVER_DRIVER_HPP
