#include <ros/ros.h>
#include <iai_qb_cube_msgs/CubeCmdArray.h>
#include <sensor_msgs/JointState.h>

class Interpolator
{
  public:
    Interpolator(const ros::NodeHandle& nh): 
        nh_(nh), run_flag_(false), p_gain_(0.0), clamp_(0.0)
    {
    }

    bool init()
    {
      if(!nh_.getParam("joint_names", joint_names_))
      {
        ROS_ERROR("No parameter 'joint_names' in namespace %s found", 
            nh_.getNamespace().c_str());
        return false;
      }

      if(!nh_.getParam("p_gain", p_gain_))
      {
        ROS_ERROR("No parameter 'p_gain' in namespace %s found", 
            nh_.getNamespace().c_str());
        return false;
      }

      if(!nh_.getParam("clamp", clamp_))
      {
        ROS_ERROR("No parameter 'p_gain' in namespace %s found", 
            nh_.getNamespace().c_str());
        return false;
      }

      pub_ = nh_.advertise<iai_qb_cube_msgs::CubeCmdArray>("out_command", 1);
      cmd_sub_ = nh_.subscribe("in_command", 1, &Interpolator::cmdCallback, this);
      state_sub_ = nh_.subscribe("in_state", 1, &Interpolator::stateCallback, this);

      return true;
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber cmd_sub_, state_sub_;

    iai_qb_cube_msgs::CubeCmdArray cmd_in_;
    bool run_flag_;
    double p_gain_, clamp_;
    std::vector<std::string> joint_names_;

    void cmdCallback(const iai_qb_cube_msgs::CubeCmdArray::ConstPtr& msg)
    {
      if(msg->commands.size() != joint_names_.size())
      {
        ROS_ERROR("Got command with too few entries: %d != %d",
	    msg->commands.size(), joint_names_.size());
        return;
      }

      for(size_t i=0; i<joint_names_.size(); i++)
        if(joint_names_[i].compare(msg->commands[i].joint_name) != 0)
        {
          ROS_ERROR("Got mismatch between joint-state and joint-command: %s != %s",
              joint_names_[i].c_str(), msg->commands[i].joint_name.c_str());
          return;
        }

      run_flag_ = true;
      cmd_in_ = *msg;
    }

    void stateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      if(run_flag_)
      {
        iai_qb_cube_msgs::CubeCmdArray out_msg;
       
        for(size_t i=0; i<msg->name.size(); ++i)
          if((i<cmd_in_.commands.size()) &&
             (msg->name[i].compare(cmd_in_.commands[i].joint_name) == 0))
          {
            iai_qb_cube_msgs::CubeCmd m;
            m = cmd_in_.commands[i];
            double error = m.equilibrium_point - msg->position[i];
            error = p_gain_ * error;
            m.equilibrium_point = msg->position[i] + clamp(error, -clamp_, clamp_);
            out_msg.commands.push_back(m);
          }

        pub_.publish(out_msg);
      }
    }

    double clamp(double num, double min, double max)
    {
      double result;
      result = num > max ? max : num;
      result = result < min ? min : result;
      return result;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iai_qb_cube_interpolator");
  ros::NodeHandle n("~");

  Interpolator my_interpolator(n);
  if(!my_interpolator.init())
    return -1;

  ros::spin();

  return 0;
}
