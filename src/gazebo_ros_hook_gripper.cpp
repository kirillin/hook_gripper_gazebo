#include "hook_gripper_gazebo/gazebo_ros_hook_gripper.hpp"

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(HookGripperPlugin)

    HookGripperPlugin::HookGripperPlugin() {}

    void HookGripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        gzmsg << "Loading " << PLUGIN_NAME << std::endl;

        this->model = _model;

        // SETUP INITIALISATION OF ATTACHING/DETACHING OBJECTS TO GRIPPER LINK
        this->attached = false;
        this->ready_to_attach = false;
        this->grasping_model_name = "";
        physics::PhysicsEnginePtr physics = this->model->GetWorld()->Physics();
        this->fixedJoint = physics->CreateJoint("revolute");


        // GET ATRIBUTES
        sdf::ElementPtr gripper_robot_name = _sdf->GetElement("gripper_robot_name");
        sdf::ElementPtr gripper_link_name = _sdf->GetElement("gripper_link_name");
        sdf::ElementPtr objects_robot_name = _sdf->GetElement("objects_robot_name");
        sdf::ElementPtr objects_link_name = _sdf->GetElement("objects_link_name");
        sdf::ElementPtr min_collision_depth_elem = _sdf->GetElement("min_collision_depth");

        if (!gripper_robot_name || !gripper_link_name || !objects_robot_name || !objects_link_name || !min_collision_depth_elem) {
            gzmsg << "Must be set all parameters! Plugin will not work!\n";
            // TODO throw exception
        } else {
            // TODO add checking for values
            this->gripper_robot_name = gripper_robot_name->Get<std::string>();
            this->gripper_link_name = gripper_link_name->Get<std::string>();
            this->objects_robot_name = objects_robot_name->Get<std::string>(); // must be complex name to be unique for grasping objects in a world
            this->objects_link_name = objects_robot_name->Get<std::string>();
            // this->min_collision_depth = 0.01;
            this->min_collision_depth = (double) min_collision_depth_elem->Get<double>();
        }
        
        //test_gripper::gripper::gripper_collision
        this->gripper_collision = this->gripper_robot_name + "::" + this->gripper_link_name + "::" + this->gripper_link_name + "_collision";
        this->objects_collision = this->objects_robot_name + "::" + this->objects_link_name + "::" + this->objects_link_name + "_collision";

        // SETUP GRIPPER
        std::string palmLinkName = this->gripper_link_name;
        this->palmLink = this->model->GetLink(palmLinkName);
    
        // SETUP COLLISIONS SUBSCRIPTION
        this->transport_node = transport::NodePtr(new transport::Node());
        this->transport_node->Init(this->model->GetWorld()->Name());
        std::string topicContactsName = "~/physics/contacts"; // /gazebo/default/physics/contacts -> gazebo.msgs.Contacts
        this->contacts_sub = this->transport_node->Subscribe(topicContactsName, &HookGripperPlugin::OnContactsMsg, this);

        // TODO publish ready_to_attach -> grasping_state
        // TODO service for attach
        // ROS initialization
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_hook_gripper_client", ros::init_options::NoSigintHandler);
        }

        ROS_INFO("Loading gazebo_hook_gripper_client");

        this->rosNode.reset(new ros::NodeHandle("gazebo_hook_gripper_client"));

        // state publisher
        ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
                                            "/" + this->gripper_link_name + "/state", 1,
                                            std::bind(&HookGripperPlugin::Connect, this),
                                            std::bind(&HookGripperPlugin::Disconnect, this),
                                            ros::VoidPtr(), &this->rosQueue);
        this->rosPubState = this->rosNode->advertise(ao);

        // service to grasp (attach/detach object)
        // ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
        //                                     "on", std::bind(&HookGripperPlugin::OnServiceCallback,
        //                                     this, _1, _2), ros::VoidPtr(), &queue_);
        // this->rosSrvGrasp = this->rosNode->advertiseService(aso1);

        this->rosQueueThread = std::thread(std::bind(&HookGripperPlugin::QueueThread, this));

        update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&HookGripperPlugin::OnUpdate, this));
        reset_connection = event::Events::ConnectWorldReset(boost::bind(&HookGripperPlugin::OnReset, this));

        gzmsg << "Loaded " << PLUGIN_NAME << std::endl;
    }

    void HookGripperPlugin::OnContactsMsg(ConstContactsPtr &_msg)
    {
        boost::mutex::scoped_lock lock(this->mutex);

        msgs::Contacts contacts;
        contacts = (*_msg);
        bool ready_to_attach = false;
        for (int i = 0; i < contacts.contact_size(); ++i)
        {
            std::string c1 = contacts.contact(i).collision1();
            std::string c2 = contacts.contact(i).collision2();

            if (c1.find(this->gripper_collision) !=  std::string::npos) {
              if (c2.find(this->objects_link_name) !=  std::string::npos) {
                for (int j = 0; j < contacts.contact(i).position_size(); ++j) {
                    if (contacts.contact(i).depth(j) >= this->min_collision_depth) {
                        ready_to_attach = true;

                        // get model (robot) name
                        std::size_t found = c2.find(":");
                        if (found !=  std::string::npos) {
                            this->grasping_model_name.clear();
                            this->grasping_model_name = c2.substr(0,found);
                            gzmsg << "Collision between: " << contacts.contact(i).collision1() << " and " << contacts.contact(i).collision1() << std::endl;
                            gzmsg << "grasping_model_name: " << this->grasping_model_name << std::endl;
                            break;
                        } else {
                            gzmsg << "Error to find model name in collision string!\n";
                        }
                    }                
                }
              }
            } else if (c2.find(this->gripper_collision) !=  std::string::npos) {
              if (c1.find(this->objects_link_name) !=  std::string::npos) {
                for (int j = 0; j < contacts.contact(i).position_size(); ++j) {
                    if (contacts.contact(i).depth(j) >= this->min_collision_depth) {
                        ready_to_attach = true;

                        // get model (robot) name
                        std::size_t found = c2.find(":");
                        if (found !=  std::string::npos) {
                            this->grasping_model_name.clear();
                            this->grasping_model_name = c2.substr(0,found);
                            gzmsg << "Collision between: " << contacts.contact(i).collision1() << " and " << contacts.contact(i).collision1() << std::endl;
                            gzmsg << "grasping_model_name: " << this->grasping_model_name << std::endl;
                            break;
                        } else {
                            gzmsg << "Error to find model name in collision string!\n";
                        }

                    }
                }
              }
            } else {
                continue;
            }
        } 
        this->ready_to_attach = ready_to_attach;       
    }

    void HookGripperPlugin::Attach() {
        boost::mutex::scoped_lock lock(this->mutex);

        physics::ModelPtr obj = this->model->GetWorld()->ModelByName(this->grasping_model_name);
        std::cerr << obj->GetLink()->WorldPose() << std::endl;
        std::cerr << this->palmLink->WorldPose() << std::endl;

        ignition::math::Pose3d diff = obj->GetLink()->WorldPose() - this->palmLink->WorldPose();

        this->fixedJoint->Reset();
        this->fixedJoint->SetName("virtual_joint");
        this->fixedJoint->AddType(gazebo::physics::Base::FIXED_JOINT);
        // this->fixedJoint->SetVelocityLimit(0,0);
        // this->fixedJoint->SetVelocityLimit(1,0);
        // this->fixedJoint->SetVelocityLimit(2,0);
        this->fixedJoint->Attach(this->palmLink, obj->GetLink());
        
        attached = !attached;
    }

    void HookGripperPlugin::Detach() {
        this->fixedJoint->Detach();
        attached = !attached;
    }

    void HookGripperPlugin::OnReset()
    {
    }

    void HookGripperPlugin::OnUpdate()
    {
        std_msgs::Bool grasping_msg;
        if (this->ready_to_attach) {
            grasping_msg.data = true;
        } else {
            grasping_msg.data = false;
        }
        this->rosPubState.publish(grasping_msg);
    }

    void HookGripperPlugin::Connect() {
        this->state_publisher_connect_count_++;
    }

    void HookGripperPlugin::Disconnect() {
        this->state_publisher_connect_count_--;
    }

    void HookGripperPlugin::QueueThread()
    {
        static const double timeout = 0.01;

        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

}
