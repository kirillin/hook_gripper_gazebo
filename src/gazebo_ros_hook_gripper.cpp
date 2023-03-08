#include "hook_gripper_gazebo/gazebo_ros_hook_gripper.hpp"

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(HookGripperPlugin)

    HookGripperPlugin::HookGripperPlugin() {}
    HookGripperPlugin::~HookGripperPlugin() 
    {
        this->update_connection.reset();

        rosQueue.clear();
        rosQueue.disable();
        rosNode->shutdown();
        rosQueueThread.join();
    }

    void HookGripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        gzmsg << "Loading " << PLUGIN_NAME << std::endl;

        this->model = _model;

        // SETUP INITIALISATION OF ATTACHING/DETACHING OBJECTS TO GRIPPER LINK
        this->attached = false;
        this->ready_to_attach = false;
        this->grasping_model_name = "";
        // physics::PhysicsEnginePtr physics = this->model->GetWorld()->Physics();
        // this->fixedJoint = physics->CreateJoint("revolute");


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
            // TODO add checking for validity of values
            this->gripper_robot_name = gripper_robot_name->Get<std::string>();
            this->gripper_link_name = gripper_link_name->Get<std::string>();
            this->objects_robot_name = objects_robot_name->Get<std::string>(); // must be complex name to be unique for grasping objects in a world
            this->objects_link_name = objects_link_name->Get<std::string>();
            // this->min_collision_depth = 0.01;
            this->min_collision_depth = (double) min_collision_depth_elem->Get<double>();
        }
        
        // this->gripper_collision = this->gripper_robot_name + "::" + this->gripper_link_name + "::" + this->gripper_link_name + "_collision";
        this->gripper_collision = this->gripper_link_name + "_collision";
        this->objects_collision = this->objects_robot_name + "::" + this->objects_link_name + "::" + this->objects_link_name + "_collision";

        // SETUP GRIPPER
        std::string palmLinkName = this->gripper_link_name;
        this->palmLink = this->model->GetLink(palmLinkName);
    
        if (!this->palmLink) // motivated by vacuum gripper
        {
            std::string found;
            physics::Link_V links = this->model->GetLinks();
            for (size_t i = 0; i < links.size(); i++) {
                found += std::string(" ") + links[i]->GetName();
            }
            ROS_FATAL_NAMED("hook_gripper", "libhook_gripper_gazebo plugin error: link named: %s does not exist", palmLinkName.c_str());
            ROS_FATAL_NAMED("hook_gripper", "libhook_gripper_gazebo plugin error: Last joint must be not fixed type!");
            ROS_FATAL_NAMED("hook_gripper", "libhook_gripper_gazebo plugin error: Found links are: %s", found.c_str());
            return;
        }
    
        // SETUP COLLISIONS SUBSCRIPTION
        this->transport_node = transport::NodePtr(new transport::Node());
        this->transport_node->Init(this->model->GetWorld()->Name());
        std::string topicContactsName = "~/physics/contacts"; // /gazebo/default/physics/contacts -> gazebo.msgs.Contacts
        this->contacts_sub = this->transport_node->Subscribe(topicContactsName, &HookGripperPlugin::OnContactsMsg, this);

        // ROS initialization
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, this->gripper_link_name, ros::init_options::NoSigintHandler);
        }

        ROS_INFO("Loading gazebo_hook_gripper_client");

        this->rosNode.reset(new ros::NodeHandle(this->gripper_link_name));

        // ready to attach publisher
        ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
                                            "/" + this->gripper_link_name + "/ready", 1,
                                            boost::bind(&HookGripperPlugin::Connect, this),
                                            boost::bind(&HookGripperPlugin::Disconnect, this),
                                            ros::VoidPtr(), &this->rosQueue);
        this->rosPubReady = this->rosNode->advertise(ao);
        this->ready_publisher_connect_count_ = 0;

        // state publisher
        ros::AdvertiseOptions ao2 = ros::AdvertiseOptions::create<std_msgs::Bool>(
                                            "/" + this->gripper_link_name + "/state", 1,
                                            boost::bind(&HookGripperPlugin::Connect2, this),
                                            boost::bind(&HookGripperPlugin::Disconnect2, this),
                                            ros::VoidPtr(), &this->rosQueue);
        this->rosPubState = this->rosNode->advertise(ao2);
        this->state_publisher_connect_count_ = 0;

        // service to grasp (attach/detach object)
        ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<std_srvs::SetBool>(
                                            "grasp", boost::bind(&HookGripperPlugin::GraspServiceCallback, this, _1, _2), 
                                            ros::VoidPtr(), &this->rosQueue);
        this->rosSrvGrasp = this->rosNode->advertiseService(aso);

        this->rosQueueThread = std::thread(std::bind(&HookGripperPlugin::QueueThread, this));

        update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&HookGripperPlugin::OnUpdate, this));

        gzmsg << "Loaded " << PLUGIN_NAME << std::endl;
    }

    // check if gripper and ok-to-grasp objects in collision
    bool HookGripperPlugin::check_collision(const std::string& c1, const std::string& c2) 
    {
        bool is_collision = false;
        if (c1.find(this->gripper_collision) !=  std::string::npos) {
            if (c2.find(this->objects_link_name) !=  std::string::npos) {
                is_collision = true;
            }
        } else if (c2.find(this->gripper_collision) !=  std::string::npos) {
            if (c1.find(this->objects_link_name) !=  std::string::npos) {
                is_collision = true;
            }
        }
        return is_collision;
    }

    // extract the grasp model name from string of names of collisions
    void HookGripperPlugin::get_grasp_model_name(std::string& model_name, const std::string &c1, const std::string &c2) 
    {
        std::string model_name_ = "";
        
        std::size_t found1 = c1.find(this->objects_link_name);
        if (found1 !=  std::string::npos) {
            std::size_t end_pos = c1.find(":");
            model_name_ = c1.substr(0, end_pos);
        } else {
            std::size_t found2 = c2.find(this->objects_link_name);
            if (found2 !=  std::string::npos) {
                std::size_t end_pos = c2.find(":");
                model_name_ = c2.substr(0, end_pos);
            } else {
                gzmsg << "Error to find any model name in collision objects!\n";
            }
        }

        model_name = model_name_;
    }

    void HookGripperPlugin::OnContactsMsg(ConstContactsPtr &_msg)
    {
        boost::mutex::scoped_lock lock(this->mutex);

        bool ready_to_attach_ = false;

        msgs::Contacts contacts;
        contacts = (*_msg);
        
        for (int i = 0; i < contacts.contact_size(); ++i)
        {
            std::string c1 = contacts.contact(i).collision1();
            std::string c2 = contacts.contact(i).collision2();
            
            // gzmsg << c1 << "\t" << c2 << std::endl;

            if (!c1.empty() && !c2.empty()) {

                // check if gripper and ok-to-grasp object in collision
                if (this->check_collision(c1, c2) || this->check_collision(c2, c1)) {
                    for (int j = 0; j < contacts.contact(i).position_size(); ++j) {
                        if (contacts.contact(i).depth(j) >= this->min_collision_depth) {
                            this->grasping_model_name.clear();
                            std::string model_name;
                            this->get_grasp_model_name(model_name, c1, c2);
                            this->grasping_model_name = model_name;
                            ready_to_attach_ = true;
                        }                
                    }
                }

            }
        } 
        this->ready_to_attach = ready_to_attach_;       
    }

    void HookGripperPlugin::Attach() 
    {

        // gzmsg << this->grasping_model_name << std::endl;
        // gzmsg << this->palmLink->GetName() << std::endl;
        
    
        physics::ModelPtr obj = this->model->GetWorld()->ModelByName(this->grasping_model_name);
        std::cerr << obj->GetLink()->WorldPose() << std::endl;
        std::cerr << this->palmLink->WorldPose() << std::endl;

        ignition::math::Pose3d diff = obj->GetLink()->WorldPose() - this->palmLink->WorldPose();

        physics::PhysicsEnginePtr physics = this->model->GetWorld()->Physics();
        this->fixedJoint = physics->CreateJoint("revolute"); // TODO move to constructor

        // this->fixedJoint->Reset();
        this->fixedJoint->SetName("virtual_joint");
        this->fixedJoint->AddType(gazebo::physics::Base::FIXED_JOINT);
        this->fixedJoint->Attach(this->palmLink, obj->GetLink());
        
        attached = !attached;
    }

    void HookGripperPlugin::Detach() {
        this->fixedJoint->Detach();
        this->fixedJoint->Fini();
        attached = !attached;
    }

    void HookGripperPlugin::OnUpdate()
    {
        
        // if ready to attach
        std_msgs::Bool grasping_msg;
        if (this->ready_to_attach) {
            grasping_msg.data = true;
        } else {
            grasping_msg.data = false;
        }
        this->rosPubReady.publish(grasping_msg);

        // if attached
        std_msgs::Bool attached_msg;
        if (this->attached) {
            attached_msg.data = true;
        } else {
            attached_msg.data = false;
        }
        this->rosPubState.publish(attached_msg);
    }

    void HookGripperPlugin::Connect() {
        this->state_publisher_connect_count_++;
    }

    void HookGripperPlugin::Disconnect() {
        this->state_publisher_connect_count_--;
    }

    void HookGripperPlugin::Connect2() {
        this->ready_publisher_connect_count_++;
    }

    void HookGripperPlugin::Disconnect2() {
        this->ready_publisher_connect_count_--;
    }

    bool HookGripperPlugin::GraspServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        boost::mutex::scoped_lock lock(this->mutex);
        
        if (req.data == true) {
            if (this->ready_to_attach && !this->attached) {
                this->Attach();
                res.success = true;
                res.message = this->grasping_model_name;
            } else {
                ROS_WARN("Can't grasp object. Move gripper closer to grasping object!");
            }
        } else {
            if (this->attached) {
                this->Detach();
                res.success = false;
                res.message = "move closer";
            }
        }
        
        return true;
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
