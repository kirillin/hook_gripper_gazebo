
#ifndef GAZEBO_ROS_HOOK_GRIPPER_HH
#define GAZEBO_ROS_HOOK_GRIPPER_HH

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

const std::string PLUGIN_NAME = "HOOK_GRIPPER_PLUGIN";

namespace gazebo
{
  class HookGripperPlugin : public ModelPlugin
  {

    public: HookGripperPlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate();
    void OnReset();

    private:
      physics::ModelPtr model;
      std::string gripperName;
      physics::JointPtr fixedJoint;
      physics::LinkPtr palmLink;

      event::ConnectionPtr update_connection;
      event::ConnectionPtr reset_connection;

      bool attached;
  };
}

#endif