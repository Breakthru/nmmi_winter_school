#include <iai_qb_cube_driver/Driver.hpp>

#include <time.h>
#include <definitions.h>

using namespace iai_qb_cube_driver;

typedef std::map<std::string, int>::iterator iterator_type;

// threading helper class
class pthread_scoped_lock
{
public:
  pthread_scoped_lock(pthread_mutex_t *mutex) : mutex_(mutex) { pthread_mutex_lock(mutex_); }
  ~pthread_scoped_lock() { unlock(); }
  void unlock() { pthread_mutex_unlock(mutex_); }
private:
  pthread_mutex_t *mutex_;
};

MotorCommand InternalCommand::toMotorCommand() const
{
  // Convert radians to degree. The minus sign is necessary to make 
  // an outgoing revolute axis.
  double position = - equilibrium_point_ * (180.0 / M_PI);

  // Convert degrees to ticks.
  short int pos = position * DEG_TICK_MULTIPLIER;
  short int stiff = stiffness_preset_ * DEG_TICK_MULTIPLIER;

  // limit equilibrium point command
  if (pos > (DEFAULT_SUP_LIMIT / pow(2, DEFAULT_RESOLUTION) - DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER)) {
    pos = (DEFAULT_SUP_LIMIT / pow(2, DEFAULT_RESOLUTION) - DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER);
  } else if (pos < (DEFAULT_INF_LIMIT / pow(2, DEFAULT_RESOLUTION) + DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER)) {
    pos = (DEFAULT_INF_LIMIT / pow(2, DEFAULT_RESOLUTION) + DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER);
  }

  // limit stiffness preset command
  if (stiff > DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER) {
    stiff = DEFAULT_STIFFNESS * DEG_TICK_MULTIPLIER;
  } else if (stiff < 0) {
    stiff = 0;
  }

  // package the result
  MotorCommand result;
  result.motor1_position_ = pos - stiff;
  result.motor2_position_ = pos + stiff;

  return result;
}

//Sends a signal to stop the rt thread
void Driver::stop_rt_thread()
{

  ROS_INFO("Telling rt thread to exit");
  pthread_mutex_lock(&mutex_);
  exitRequested_ = true;
  pthread_mutex_unlock(&mutex_);

  pthread_join(thread_, 0);
}

//Sets up and starts the rt thread
bool Driver::start_rt_thread(double timeout){
  timeout_ = timeout;
  exitRequested_ = false;

  // setting up mutex
  pthread_mutexattr_t mattr;
  pthread_mutexattr_init(&mattr);
  pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT);

  pthread_mutex_init(&mutex_,  &mattr);

  // setting up thread with high priority
  pthread_attr_t tattr;
  struct sched_param sparam;
  sparam.sched_priority = 12;
  pthread_attr_init(&tattr);
  pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);
  pthread_attr_setschedparam(&tattr, &sparam);
  pthread_attr_setinheritsched (&tattr, PTHREAD_EXPLICIT_SCHED);

  if(pthread_create(&thread_, &tattr, &Driver::run_s, (void *) this) != 0)
  {
    fprintf(stderr, "# ERROR: could not create realtime thread\n");
    return false;
  }

  return true;
}

//Actual function runing in the rt thread
void* Driver::rt_run()
{
  while(!exitRequested_) {
    readMeasurement(); 
    writeMeasurementBuffer(measurement_tmp_);

    readCommandBuffer(); 
    writeCommand();
    if (simulationMode())
    	usleep(1000);
  }

  ROS_INFO("exiting rt thread");
  //TODO: should we stop the modules and communiction here?
}

Driver::Driver(const ros::NodeHandle& nh): 
    nh_(nh), comm_up_(false), cubes_active_(false), sim_mode_(false)
{
    //set up the mutex
    pthread_mutexattr_t mattr;
    pthread_mutexattr_init(&mattr);
    pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT);

    pthread_mutex_init(&mutex_,  &mattr);
}

Driver::~Driver()
{
  deactivateCubes();
  stopCubeCommunication();
}

void Driver::run()
{
  if(!readParameters())
      return;

  initDatastructures();

  if(!simulationMode())
  {
    if (!startCubeCommunication())
      return;
 
    if(!activateCubes())
      return;
  }

  if (!start_rt_thread(2))
      return;
 
  cmd_sub_ = nh_.subscribe("command", 1, &Driver::cmd_sub_cb_, this);

  ros::AsyncSpinner spinner(1);
  spinner.start();


  ros::Rate r(publish_frequency_);
  while(ros::ok())
  {
    pub_.publish(getJointStateMsg());
    cube_pub_.publish(getCubeStateMsg());
 
    r.sleep();
  }

  //Exit cleanly
  this->stop_rt_thread();
}

void Driver::cmd_sub_cb_(const iai_qb_cube_msgs::CubeCmdArray::ConstPtr& msg)
{
  std::vector<InternalCommand> desired_command;
  for(size_t i=0; i<msg->commands.size(); i++)
  {
    if(cube_id_map_.count(msg->commands[i].joint_name) == 1)
    {
      // assert: joint_name is known to us
      InternalCommand new_command;
      new_command.cube_id_ = cube_id_map_[msg->commands[i].joint_name];
      new_command.equilibrium_point_ = msg->commands[i].equilibrium_point;
      new_command.stiffness_preset_ = msg->commands[i].stiffness_preset;
      desired_command.push_back(new_command);
    }
    else
      ROS_WARN("Asked to command unknown joint %s", msg->commands[i].joint_name.c_str());
  }

  ROS_DEBUG("New setpoints.");

  writeCommandBuffer(desired_command);
}

sensor_msgs::JointState Driver::getJointStateMsg()
{

  sensor_msgs::JointState msg;
  std::vector<InternalState> measurement = readMeasurementBuffer();

  assert(cube_id_map_.size() == measurement.size());

  msg.header.stamp = ros::Time::now();

  size_t i=0;
  for (iterator_type it=cube_id_map_.begin();  
       it!=cube_id_map_.end(); ++it)
  { 
    if(measurement[i].cube_id_ == it->second)
    {
      msg.name.push_back(it->first);
      msg.position.push_back(measurement[i].joint_position_);
    }
    else
    {
      ROS_WARN("Mismatching cube-id during construction of JointState message.");
      ROS_WARN("measurement[i].cube_id_ == %d", measurement[i].cube_id_);
      ROS_WARN("it->second == %d", it->second);
    }
    ++i;
  }

  return msg;
}

iai_qb_cube_msgs::CubeStateArray Driver::getCubeStateMsg()
{
  iai_qb_cube_msgs::CubeStateArray msg;
  std::vector<InternalState> measurement = readMeasurementBuffer();

  assert(cube_id_map_.size() == measurement.size());

  msg.header.stamp = ros::Time::now();

  size_t i=0;
  for (iterator_type it=cube_id_map_.begin();  
       it!=cube_id_map_.end(); ++it)
  { 
    if(measurement[i].cube_id_ == it->second)
    {
      iai_qb_cube_msgs::CubeState m;
      m.joint_name = it->first;
      m.cube_id = measurement[i].cube_id_;
      m.pos_motor1 = measurement[i].motor1_position_;
      m.pos_motor2 = measurement[i].motor2_position_;
      m.pos_joint = measurement[i].joint_position_;
      msg.states.push_back(m);
    }
    else
    {
      ROS_WARN("Mismatching cube-id during construction of CubeState message.");
      ROS_WARN("measurement[i].cube_id_ == %d", measurement[i].cube_id_);
      ROS_WARN("it->second == %d", it->second);
    }
    ++i;
  }

  return msg;
}

bool Driver::startCubeCommunication()
{

  //return true; //For now, to test the RT parts
  if(isCommunicationUp())
    return true;

  openRS485(&cube_comm_, port_.c_str());

  if (cube_comm_.file_handle <= 0) {
    ROS_ERROR("Could not connect to QBCubes");
    return false;
  } else {
    ROS_INFO("Started communication to QBCubes");
    comm_up_ = true;
  }

  return true;
}

void Driver::stopCubeCommunication()
{
  if(!isCommunicationUp())
    return;

  closeRS485(&cube_comm_);
}

void Driver::readMeasurement()
{
  // In simulation mode the member writeCommand(...) will update the internal
  // measurement datastructures. Hence, nothing needs to be done here.
  if(!simulationMode())
  {
    assert(cube_id_map_.size() == measurement_tmp_.size());
  
    // TODO: possibly speed me up by not looking up in the map..
    size_t i = 0;
    for (iterator_type it=cube_id_map_.begin();  
         it!=cube_id_map_.end(); ++it)
    { 
      short int cube_id = it->second;
      short int measurements[3];
      commGetMeasurements(&cube_comm_, cube_id, measurements);
      // all measurements in radians
      // TODO: 1, too, many...
      measurement_tmp_[i].motor1_position_ = 
          - (measurements[0]/DEG_TICK_MULTIPLIER)*(M_PI/180);
      measurement_tmp_[i].motor2_position_ = 
          - (measurements[1]/DEG_TICK_MULTIPLIER)*(M_PI/180);
      measurement_tmp_[i].joint_position_ = 
          - (measurements[2]/DEG_TICK_MULTIPLIER)*(M_PI/180);
  
      i++;
    }
  }
}

void Driver::writeCommand()
{
  //write to the modules
  unsigned int i = 0;
  for (size_t i=0; i<command_buffer_.size(); i++)
    commandSingleCube(command_buffer_[i]);
}

void Driver::commandSingleCube(const InternalCommand& command)
{
  MotorCommand motor_cmd = command.toMotorCommand();
  
  if(simulationMode())
  {
    size_t index = 0; // TODO(fix me)
    for (iterator_type it=cube_id_map_.begin(); it!=cube_id_map_.end(); ++it)
    {
      if(it->second == command.cube_id_) break;
      ++index;
    }
 
    measurement_tmp_[index].cube_id_ = command.cube_id_;
    measurement_tmp_[index].joint_position_ = command.equilibrium_point_;
    measurement_tmp_[index].motor1_position_ = 
        - (motor_cmd.motor1_position_/DEG_TICK_MULTIPLIER)*(M_PI/180);
    measurement_tmp_[index].motor2_position_ = 
        - (motor_cmd.motor2_position_/DEG_TICK_MULTIPLIER)*(M_PI/180);
  }
  else
  {
    short int curr_ref[2];
    curr_ref[0] = motor_cmd.motor1_position_;
    curr_ref[1] = motor_cmd.motor2_position_;

    commSetInputs(&cube_comm_, command.cube_id_, curr_ref);
  }
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

  if(!nh_.getParam("sim_mode", sim_mode_))
  {
    ROS_ERROR("No parameter 'sim_mode' in namespace %s found", 
        nh_.getNamespace().c_str());
    return false;
  }

  std::vector<std::string> joint_names;
  if(!nh_.getParam("joint_names", joint_names))
  {
    ROS_ERROR("No parameter 'joint_names' in namespace %s found", 
        nh_.getNamespace().c_str());
    return false;
  }

  cube_id_map_.clear();
  for (size_t i=0; i<joint_names.size(); i++)
  {
    int cube_id; 
    if(!nh_.getParam(joint_names[i] + "/id", cube_id))
    {
      ROS_ERROR("No parameter '%s/id' in namespace %s found", 
          joint_names[i].c_str(), nh_.getNamespace().c_str());
      return false;
    }
    cube_id_map_.insert(std::pair<std::string, int>(joint_names[i], cube_id));
  }

  std::cout << "Starting cubes in ";
  if (sim_mode_)
    std::cout << "simulation mode\n";
  else
    std::cout << "real-world mode\n";

  std::cout << "joint-names:\n";
  for (size_t i=0; i<joint_names.size(); i++)
    std::cout << joint_names[i] << "\n";

  std::cout << "joint-name -> cube-id:\n";
  for (iterator_type it=cube_id_map_.begin();  
       it!=cube_id_map_.end(); ++it)
    std::cout << it->first << " => " << it->second << '\n';

  return true;
}

bool Driver::activateCubes()
{
  if(!isCommunicationUp())
    return false;

  // We are activating the cubes regardless of whether we think we
  // already have them activated. Can't really hurt.

  for(iterator_type it = cube_id_map_.begin(); it != cube_id_map_.end(); it++) 
  {
    char tmp = 0x00;
    short int cube_id = it->second;
    ROS_INFO("Trying to activate cube with id '%d'...", cube_id);
    while(tmp == 0x00) 
    {
      if (cube_id > 30) break; // Georg wonders why 30?
      commActivate(&cube_comm_, cube_id, 1);
      usleep(100000);
      commGetActivate(&cube_comm_, cube_id, &tmp);
      ROS_INFO("tmp: %d", tmp);
      usleep(100000);
    }

    ROS_INFO("...succeeded");
  }

  cubes_active_ = true;
	
  return true;
}

void Driver::deactivateCubes()
{
  if(!isCommunicationUp())
    return;

  // We deactivate the cubes regardless of whether we think we
  // already have them deactivated. Better safe than sorry.

  for(iterator_type it = cube_id_map_.begin(); it != cube_id_map_.end(); it++) 
  {
    char tmp = 0x00;
    short int cube_id = it->second;
    ROS_INFO("Deactivating cube with id '%d'", cube_id);
    while(tmp == 0x00) 
    {
      if (cube_id > 30) break; // Georg wonders why 30?
      commActivate(&cube_comm_, cube_id, 0);
    }
  }

  cubes_active_ = false;
}

void Driver::initDatastructures()
{
  pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 1);
  cube_pub_ = nh_.advertise<iai_qb_cube_msgs::CubeStateArray>("cube_state", 1);

  measurement_buffer_.resize(cube_id_map_.size());
  measurement_tmp_.resize(cube_id_map_.size());
  size_t i=0;
  for(iterator_type it = cube_id_map_.begin(); it != cube_id_map_.end(); ++it) 
  {
    measurement_buffer_[i].cube_id_ = it->second; 
    measurement_tmp_[i].cube_id_ = it->second; 
    ++i;
  }
}

const std::vector<InternalCommand>& Driver::readCommandBuffer()
{
  pthread_scoped_lock scoped_lock(&mutex_);
  return command_buffer_;
}

void Driver::writeCommandBuffer(const std::vector<InternalCommand>& new_command)
{
  pthread_scoped_lock scoped_lock(&mutex_);
  command_buffer_ = new_command;
}

const std::vector<InternalState>& Driver::readMeasurementBuffer()
{
  pthread_scoped_lock scoped_lock(&mutex_);
  return measurement_buffer_;
}

void Driver::writeMeasurementBuffer(const std::vector<InternalState>& new_measurement)
{
  pthread_scoped_lock scoped_lock(&mutex_);
  measurement_buffer_ = new_measurement;
}
