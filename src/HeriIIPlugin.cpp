#include <rosee_gazebo_plugins/HeriIIPlugin.h>

void gazebo::HeriIIPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
 
    // Store the pointer to the model
    this->model = _parent;
    
    // Error message if the model couldn't be found
    if (!model) {
        std::cout << "Model is NULL! HeriIIPlugin could not be loaded.";
        return;
    
    }
    
    // Initialize ros, if it has not already been initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "heriII_gazebo_plugin", ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. 
    this->rosNode.reset(new ros::NodeHandle("heriII_gazebo_plugin"));
    
    std::string configFileName;

    if (! rosNode->getParam("/ros_ee_config_path", configFileName) ) {
        ROS_ERROR_STREAM ("[ERROR HERI gazebo plugin]: parameter '/ros_ee_config_path'" <<
            " not loaded on parameter server");
        return ;
    }
    

    YAML::Node node = YAML::LoadFile(configFileName);

    //TODO check if info are present in yaml?
    for(YAML::const_iterator moto_it = node["Motor_actuation_info"].begin(); moto_it != node["Motor_actuation_info"].end(); ++moto_it) {
        
        auto values = moto_it->second;
        
        std::vector <std::string> associatedJoints;
        associatedJoints.push_back(values["joint_1"].as<std::string>());
        associatedJoints.push_back(values["joint_2"].as<std::string>());
        if (values["joint_3"]) { //no 3rd phalange for thumb
            associatedJoints.push_back(values["joint_3"].as<std::string>());
        }
        
        MotorConfig motorConfig {values["motor_urdf_name"].as<std::string>(), associatedJoints};

        motorConfig.p = values["pid"]["p"].as<double>();
        motorConfig.i = values["pid"]["i"].as<double>();
        motorConfig.d = values["pid"]["d"].as<double>();
        motorConfig.K_t = values["K_t"].as<double>();
        motorConfig.r = values["r"].as<double>();
        motorConfig.R_M = values["R_M"].as<double>();
        motorConfig.G_r = values["G_r"].as<double>();
        motorConfig.E_e = values["E_e"].as<double>();
        
        
        if (! model->GetJoint(motorConfig.name) ) {
            ROS_ERROR_STREAM ("[ERROR HERI gazebo plugin]: motor '" << motorConfig.name << 
                "' not found as a joint in the urdf. You should add it as virtual joint" << 
                " so Gazebo can use PID on it");
            ROS_ERROR_STREAM ("Joint list:");
            for (auto it : model->GetJoints() ) {
                std::cout << "\t" << it->GetName() << std::endl;
            }
            ROS_ERROR_STREAM ("Joint list:");
            for (auto it : model->GetJoints() ) {
                std::cout << "\t" << it->GetName() << std::endl;
            }
            return ;
        }
        
        //gazebo pid to move the motors
        this->model->GetJointController()->SetPositionPID(
                model->GetJoint(motorConfig.name)->GetScopedName(), common::PID(motorConfig.p, motorConfig.d, motorConfig.d));
        
        motorsConfigs.insert( std::make_pair ( motorConfig.name , motorConfig));

    }
    
    
    //TODO take this from some config file or launch file?
    //it is named joint_commands but in truth motor position commands are sent
    std::string commandTopic = "/ros_end_effector/joint_commands";
    
    
    rosSub = rosNode->subscribe(commandTopic, 1, &gazebo::HeriIIPlugin::motorCommandClbk, this);
    
    rosPub = rosNode->advertise < sensor_msgs::JointState > ( "/rosee_gazebo_plugins/joint_states", 1 ) ;
    newCommand = false;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&HeriIIPlugin::OnUpdate, this));
    
}
    
void gazebo::HeriIIPlugin::OnUpdate()
{

    //this->model->Update();
    
    if (newCommand) {
        
        for (auto it : motorsPositionCommand) {
            std::string motorScopedName = this->model->GetJoint(it.first)->GetScopedName();
            this->model->GetJointController()->SetPositionTarget (
                motorScopedName, it.second );
            
            ROS_WARN_STREAM ( this->model->GetJointController()->GetPositionPIDs().at(motorScopedName).GetCmd());

        }
        newCommand = false;
    }
    
    //here we move the finger joints
    
    for (auto motorConfigPair : motorsConfigs) {
        
        MotorConfig motorConfig = motorConfigPair.second;

        //double theta_d = it.second;
        //0: the axis, 0 is the one we want
        //double theta = this->model->GetJoint(it.first)->GetAngle(0).Radian();
        
        std::string motorScopedName = this->model->GetJoint(motorConfig.name)->GetScopedName();
        
        double cmd = this->model->GetJointController()->GetPositionPIDs().at(motorScopedName).GetCmd();
        
        double f_tendon = 
            motorConfig.G_r * motorConfig.E_e * motorConfig.K_t * cmd
            / (motorConfig.r * motorConfig.R_M);

        double joint_DeltAngle[motorConfig.linked_joints.size()];
        joint_DeltAngle[0] = JDA.DeltAngle_Join1(f_tendon);
        joint_DeltAngle[1] = JDA.DeltAngle_Join2(f_tendon);
        
        if (motorConfig.linked_joints.size() > 2 ) { //no 3rd phalange for thumb
            joint_DeltAngle[2] = JDA.DeltAngle_Join3(f_tendon);
        }
        
        //Be sure joints in the motorConfig.linked_joints are in order (they are because how we parse the yaml)
        int i = 0;
        for (auto joint : motorConfig.linked_joints) {
            model->GetJoint(joint)->SetPosition(0, joint_DeltAngle[i] ); //0 is the joint axis
            i++;
        }
    }
    
    
    pubJointStates();
}

void gazebo::HeriIIPlugin::pubJointStates() {
    
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    
    for (auto joint : model->GetJoints()) {
        
        //HACK TODO I do not know why but fixed joint is seen as the other revolute
        if (joint->GetName().compare("world_base") == 0) {
            continue;
        }
        
        //type of joint is a sum: so fixed is joint+fixed_joint (0x4000 + 0x40 = 0x4040)
        if (joint->GetType() != 
                (gazebo::physics::Entity::JOINT + gazebo::physics::Entity::FIXED_JOINT) ) {
        
        
        //if (joint->GetName().compare("motor_finger1") == 0) { //debug
                msg.name.push_back(joint->GetName());
                msg.position.push_back(joint->GetAngle(0).Radian()); //index 0 is the "right" axis
                //msg.velocity.push_back(joint->GetVelocity(0)); //index 0 is the "right" axis
                //WARNING getForce not yet implemented (in gazebo7 is an empty function...)
                //msg.effort.push_back(joint->GetForce(0)); //index 0 is the "right" axis
            }
        //}
    }
    
    rosPub.publish(msg);

}

void gazebo::HeriIIPlugin::motorCommandClbk(const sensor_msgs::JointStateConstPtr& msg) {
    
    for (int i = 0; i < msg->name.size(); i++) { 

        motorsPositionCommand [msg->name.at(i)] = msg->position.at(i);
    }
    
    newCommand = true;
    
}
