#include "hook_gripper_gazebo/gazebo_ros_hook_gripper.hpp"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(HookGripperPlugin)

    HookGripperPlugin::HookGripperPlugin() {}

    void HookGripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        gzmsg << "Loading " << PLUGIN_NAME << std::endl;

        this->model = _model;

        std::string palmLinkName = "gripper";
        this->palmLink = this->model->GetLink(palmLinkName);

        
        update_connection = event::Events::ConnectWorldUpdateEnd(boost::bind(&HookGripperPlugin::OnUpdate, this));
        reset_connection = event::Events::ConnectWorldReset(boost::bind(&HookGripperPlugin::OnReset, this));

        attached = false;
        physics::PhysicsEnginePtr physics = this->model->GetWorld()->Physics();
        this->fixedJoint = physics->CreateJoint("revolute");
    }

    void HookGripperPlugin::OnReset()
    {
        gzmsg << "Reset!" << std::endl;

        if (!attached) {
            attached = !attached;
            physics::ModelPtr obj = this->model->GetWorld()->ModelByName("box");
            std::cerr << obj->GetLink()->WorldPose() << std::endl;
            std::cerr << this->palmLink->WorldPose() << std::endl;

            ignition::math::Pose3d diff = obj->GetLink()->WorldPose() - this->palmLink->WorldPose();
            std::cerr << diff << std::endl;

            // this->fixedJoint = this->model->CreateJoint("virtual_joint","fixed", this->palmLink, obj->GetLink());
            
            this->fixedJoint->Reset();
            this->fixedJoint->SetName("virtual_joint");
            this->fixedJoint->AddType(gazebo::physics::Base::FIXED_JOINT);
            this->fixedJoint->Attach(this->palmLink, obj->GetLink());
        } else {
            attached = !attached;
            this->fixedJoint->Detach();
        }
    }

    void HookGripperPlugin::OnUpdate()
    {
        // gzmsg << "Update!" << std::endl;
    }

}
