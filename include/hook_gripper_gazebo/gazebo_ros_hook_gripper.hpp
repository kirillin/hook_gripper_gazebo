
#ifndef GAZEBO_ROS_HOOK_GRIPPER_HH
#define GAZEBO_ROS_HOOK_GRIPPER_HH

#include <string>

#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include "std_msgs/Bool.h"
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>


const std::string PLUGIN_NAME = "HOOK_GRIPPER_PLUGIN";

namespace gazebo
{
  class HookGripperPlugin : public ModelPlugin
  {

    public: 
      HookGripperPlugin();
      virtual ~HookGripperPlugin();
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void OnContactsMsg(ConstContactsPtr &_msg);
      void Attach();
      void Detach();
      void OnUpdate();
      void OnReset();   

    private:
      std::string gripper_robot_name;
      std::string gripper_link_name;
      std::string objects_robot_name;
      std::string objects_link_name;
      std::string gripper_collision;
      std::string objects_collision;
      std::string grasping_model_name;
      std::string gripperName;

      physics::ModelPtr model;
      physics::JointPtr fixedJoint;
      physics::LinkPtr palmLink;

      gazebo::transport::NodePtr transport_node;
      gazebo::transport::SubscriberPtr contacts_sub;

      event::ConnectionPtr update_connection;

      mutable boost::mutex mutex;
      double min_collision_depth;
      bool ready_to_attach;
      bool attached;

      bool check_collision(const std::string& c1, const std::string& c2);
      void get_grasp_model_name(std::string& model_name, const std::string& c1, const std::string& c2);

      //  ROS stuff
      std::unique_ptr<ros::NodeHandle> rosNode;
      
      ros::Publisher rosPubReady;
      int ready_publisher_connect_count_;

      ros::Publisher rosPubState;
      int state_publisher_connect_count_;

      ros::ServiceServer rosSrvGrasp;
      ros::CallbackQueue rosQueue;
      std::thread rosQueueThread;

      void QueueThread();
      void Connect();
      void Disconnect();
      void Connect2();
      void Disconnect2();
      bool GraspServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  };
}

#endif