#include <iai_qb_cube_driver/Driver.hpp>

#include <time.h>

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
        //Read state from the modules


        //lock and write to the in buffer


        //lock and read from the out-buffer

        //write to the modules
        ROS_WARN("rt_run() got called");
        usleep(100000);

    }

    ROS_INFO("exiting rt thread");

    //TODO: should we stop the modules and communiction here?

}


Driver::Driver(const ros::NodeHandle& nh): nh_(nh), comm_up_(false)
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


  if (!startCubeCommunication())
    return;

  if (!start_rt_thread(2))
      return;

  if(!activateCubes())
    return;


  cmd_sub_ = nh_.subscribe("command", 1, &Driver::cmd_sub_cb_, this);

  ros::AsyncSpinner spinner(1);
  spinner.start();


  ros::Rate r(publish_frequency_);
  while(ros::ok())
  {
    readMeasurements(); 
    msg_.header.stamp = ros::Time::now();
    pub_.publish(msg_);
    r.sleep();
  }

  //Exit cleanly
  this->stop_rt_thread();


}

void Driver::cmd_sub_cb_(const iai_qb_cube_msgs::CubeCmd::ConstPtr& msg)
{

    unsigned int len_id_map = cube_id_map_.size();

    //TODO: Guard the writing
    //TODO: replace this by comparing the names of the joints
    unsigned int len_eq = msg->eq_point_angle.size();
    if (len_eq == len_id_map) {
        for (unsigned int i = 0; i < len_id_map ; ++i) {
            des_joint_eqpoints[i] = msg->eq_point_angle[i];
        }
    }

    unsigned int len_st = msg->stiffness.size();
    if (len_st == len_id_map) {
        for (unsigned int i = 0; i < len_id_map ; ++i) {
            des_joint_stiffness[i] = msg->stiffness[i];
        }
    }

    ROS_WARN("New setpoints.");

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

  if(!nh_.getParam("joint_names", msg_.name))
  {
    ROS_ERROR("No parameter 'joint_names' in namespace %s found", 
        nh_.getNamespace().c_str());
    return false;
  }

  cube_id_map_.clear();
  for (size_t i=0; i<msg_.name.size(); i++)
  {
    int cube_id; 
    if(!nh_.getParam(msg_.name[i] + "/id", cube_id))
    {
      ROS_ERROR("No parameter '%s/id' in namespace %s found", 
          msg_.name[i].c_str(), nh_.getNamespace().c_str());
      return false;
    }
    cube_id_map_.insert(std::pair<std::string, int>(msg_.name[i], cube_id));
  }

  std::cout << "joint-names:\n";
  for (size_t i=0; i<msg_.name.size(); i++)
    std::cout << msg_.name[i] << "\n";

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
  msg_.position.resize(msg_.name.size());

  pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 1);

  //Get number of joints
  int numjoints = cube_id_map_.size();

  //resize the variables for storage
  des_joint_eqpoints.resize(numjoints);
  des_joint_stiffness.resize(numjoints);

  //initialize with zeros
  for (unsigned int i = 0; i< numjoints; ++i) {
      des_joint_eqpoints[i] = 0.0;
      des_joint_stiffness[i] = 0.0;
  }
}
