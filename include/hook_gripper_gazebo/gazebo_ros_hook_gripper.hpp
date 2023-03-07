
#ifndef GAZEBO_ROS_HOOK_GRIPPER_HH
#define GAZEBO_ROS_HOOK_GRIPPER_HH

#include <string>

#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// #include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Bool.h"


const std::string PLUGIN_NAME = "HOOK_GRIPPER_PLUGIN";

namespace gazebo
{
  class HookGripperPlugin : public ModelPlugin
  {

    public: 
      HookGripperPlugin();
      // TODO add destructor
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void OnContactsMsg(ConstContactsPtr &_msg);
      void Attach();
      void Detach();

      void OnUpdate();
      void OnReset();   

    private:
      // parameters from sdf
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
      event::ConnectionPtr reset_connection;

      mutable boost::mutex mutex;
      double min_collision_depth;
      bool ready_to_attach;
      bool attached;

      //  ROS stuff
      std::unique_ptr<ros::NodeHandle> rosNode;
      
      ros::Publisher rosPubState;
      int state_publisher_connect_count_;

      ros::ServiceServer rosSrvGrasp;
      ros::CallbackQueue rosQueue;
      std::thread rosQueueThread;

      void QueueThread();
      void Connect();
      void Disconnect();
      // bool OnServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  };
}

#endif