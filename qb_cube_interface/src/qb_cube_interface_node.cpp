// ROS libraries
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <ros/console.h>

// Standard libraries
#include <time.h>
#include <boost/thread/mutex.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <string.h>

// Cube libraries
#include "qbmove_communications.h"
#include "../../../../devel/include/qb_cube_interface/SetCubePosStiff.h"
#include "qb_cube_interface/SetCubePosStiff.h"

#define NUM_OF_IDS 30

/* Definition of class QBCube */

class QBCube
{
public:
  QBCube();
  virtual ~QBCube();

private:

  ros::NodeHandle nh_; // ROS's NodeHandle.

  std::string port_; // Port at which the cube is connected, generally /dev/ttyUSB0.

  ros::ServiceServer srv_cube_pos_; // ROS's server that receive the position.

  /* Function that is called when the server receive a request. It's the main
  /* function that send the position. */ 
  bool setCubePos(qb_cube_interface::SetCubePosStiff::Request  &req,
                        qb_cube_interface::SetCubePosStiff::Response &res );
  double encoderRate_;

  // Cube stuff.
  bool isConnected_;
  int  current_skip_counter_, current_skip_max_;
  short int current_threshold_;
  boost::mutex cube_mutex_;
  std::map<int,bool> cubes;
  
  // Base time for messages timestamp.
  double start_time;

  // Utility functions for the comunication.  
  comm_settings cube_comm_;
  int connectToCube();
  void getCubeStatus();
  void closeComm();
  int setPosStiff(short int pos, short int stiff, short int cube_id); 
};

  // Class contructor. 
	QBCube::QBCube()
  {
  // Get the start time.
		start_time = ros::Time::now().toSec();
  	
		std::cerr<<"starting node\n";
		nh_ = ros::NodeHandle("~");
		nh_.param<std::string>("port", port_,"/dev/ttyUSB0");
		nh_.param<double>("encoderRate", encoderRate_, DEG_TICK_MULTIPLIER);
		 
		int tmp;
		nh_.param<int>("current_threshold_", tmp, 10000);
		current_threshold_ = tmp;

	// Server advertise. 
		srv_cube_pos_ = nh_.advertiseService("set_cube_pos", &QBCube::setCubePos, this);

		isConnected_ = false;

		std::cerr << "connecting to cube at " << port_  << std::endl;

		if (this->connectToCube()) {
		  ROS_INFO("Cube connected! ;)");  
		}
  }

	// Class distructor.
	QBCube::~QBCube()
	{
  	this->closeComm();
	}
  
/* Function that perform the connection of the cube */
int QBCube::connectToCube() {
  if(isConnected_) return 1;

  cube_mutex_.lock();

  std::cerr<<"open port\n";

  openRS485(&cube_comm_, port_.c_str());

  std::cerr<<"done\n";

  if (cube_comm_.file_handle <= 0) {
    ROS_WARN("Panic, cube file handle was invalid");
    cube_mutex_.unlock();
    return 0;
  } else {
    ROS_INFO_STREAM("Opened cube_comm_ with file handle " << cube_comm_.file_handle);
    cube_mutex_.unlock();
    isConnected_ = true;
    return 1;
  }   
}

/* Function that physically move the cube */
bool QBCube::setCubePos(qb_cube_interface::SetCubePosStiff::Request  &req, 
            qb_cube_interface::SetCubePosStiff::Response &res ){
            
  double position;
  short int stiffness;
  short int stiff_input;
  short int cube_id;
  short int pos_input;
  short int opening_measurements[3];
	res.cube_id = std::vector<int32_t> (req.cube_id.size(),0);
	res.position = std::vector<double> (req.cube_id.size(),0);
	res.timestamp = std::vector<double> (req.cube_id.size(),0);
	
  if(!isConnected_) {
    ROS_ERROR("Error: Cube not connected");
    return false;
  }

	for(int i = 0; i < req.cube_id.size(); i++){
		
		cube_id = req.cube_id.at(i);
		position = req.position.at(i);
		stiffness = req.stiffness.at(i);
		
		ROS_INFO_STREAM("id: "<<cube_id<<" pos: "<<position<<" stiff: "<<stiffness<<"\n");
		
		if(cubes.count(cube_id) == 0) {
		  cubes[cube_id] = true;
		  char tmp = 0x00;

			// Cube activation.
		  printf("Activating cube id #: %d\n", cube_id);
		  while(tmp == 0x00) {
		    if (cube_id > 30) break;
		    commActivate(&cube_comm_, cube_id, 1);
		    usleep(100000);
		    commGetActivate(&cube_comm_, cube_id, &tmp);
		    usleep(100000);
		  }
		  printf("DONE\n");
		
			// Retrive measurements from the cube.
		  commGetMeasurements(&cube_comm_, cube_id, opening_measurements);
		  ROS_INFO_STREAM("Connected to cube and read: "<< -(double)opening_measurements[2] / encoderRate_);
		}

		cube_mutex_.lock();
		  
		// Discriminating if we want to command cubes or we want only read encoder measurements or move the hand //
		if(req.command == "write"){ 
		  
			// Convert radians to degree. The minus sign is necessary to make an outgoing
			// revolute axis.
			position = -position * (180.0 / M_PI);

			// Convert degrees to ticks.
			pos_input = position * encoderRate_;
			stiff_input = encoderRate_*stiffness;

			// Stiffness and position set.
			setPosStiff(pos_input, stiff_input, cube_id);
		
		  commGetMeasurements(&cube_comm_, cube_id, opening_measurements);
		  		
			res.position.at(i) =  -(opening_measurements[2]/encoderRate_)*(M_PI/180); // Position in radians.
			res.cube_id.at(i) = cube_id;
			res.timestamp.at(i) = ros::Time::now().toSec() - start_time;	
		}
		else if(req.command == "read"){
			// Updating the measurements that are send back.
			commGetMeasurements(&cube_comm_, cube_id, opening_measurements);
			res.position.at(i) = -(opening_measurements[2]/encoderRate_)*(M_PI/180); // Position in radians.
			res.cube_id.at(i) = cube_id;
			res.timestamp.at(i) = ros::Time::now().toSec() - start_time;	
		}
		else{
			// Open or close hand.
			// In this case, position is in [0,1] and describe how much the hand is close
			// position = 0 means open, 1 complete close.
			short int curr_ref[NUM_OF_MOTORS];
			
			curr_ref[0] = position*15000;
			curr_ref[1] = position*15000;
		
			commSetInputs(&cube_comm_, cube_id, curr_ref);
		}
		cube_mutex_.unlock();
  }

  return true;
}

// Function that close the comunication.
void QBCube::closeComm() {
    if(!isConnected_) return;
    cube_mutex_.lock();
    closeRS485(&cube_comm_);
    cube_mutex_.unlock();
}

/* Function that consider the physical limits of the cube and send the angular
/* position to the engine using stiffness and position */
int QBCube::setPosStiff(short int pos, short int stiff, short int cube_id) {

  short int curr_ref[NUM_OF_MOTORS];

  if (pos > (DEFAULT_SUP_LIMIT / pow(2, DEFAULT_RESOLUTION) - DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER)) {
    pos = (DEFAULT_SUP_LIMIT / pow(2, DEFAULT_RESOLUTION) - DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER);
  } else if (pos < (DEFAULT_INF_LIMIT / pow(2, DEFAULT_RESOLUTION) + DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER)) {
    pos = (DEFAULT_INF_LIMIT / pow(2, DEFAULT_RESOLUTION) + DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER);
  }

  if (stiff > DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER) {
    stiff = DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER;
  } else if (stiff < 0) {
    stiff = 0;
  }
	
	// Position for the 2 engine of the cube.
  curr_ref[0] = pos - stiff;
  curr_ref[1] = pos + stiff;

  commSetInputs(&cube_comm_, cube_id, curr_ref);

  return 1;
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "cube_interface_node");
    QBCube cubeNode;
    ros::spin();
    return 0;
}
