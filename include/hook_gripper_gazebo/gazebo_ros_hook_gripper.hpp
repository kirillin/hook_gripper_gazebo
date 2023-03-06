
#ifndef GAZEBO_ROS_HOOK_GRIPPER_HH
#define GAZEBO_ROS_HOOK_GRIPPER_HH

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
// #include <gazebo/util/system.hh>


const std::string PLUGIN_NAME = "HOOK_GRIPPER_PLUGIN";

namespace gazebo
{
  class HookGripperPlugin : public ModelPlugin
  {

    public: 
    
      HookGripperPlugin();
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void OnUpdate();
      void OnReset();

      gazebo::transport::NodePtr transport_node;
      gazebo::transport::SubscriberPtr contacts_sub;

      // void OnMsg(ConstContactsPtr &_msg);
      void cb(ConstWorldStatisticsPtr &_msg);

    private:

      std::string gripperName;

      physics::ModelPtr model;
      physics::JointPtr fixedJoint;
      physics::LinkPtr palmLink;

  
      event::ConnectionPtr update_connection;
      event::ConnectionPtr reset_connection;

      bool attached;
  };
}

#endif