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
        
        MotorConfig motorConfig {moto_it->first.as<std::string>(), associatedJoints};

        motorConfig.p = values["pid"]["p"].as<double>();
        motorConfig.i = values["pid"]["i"].as<double>();
        motorConfig.d = values["pid"]["d"].as<double>();
        motorConfig.K_t = values["K_t"].as<double>();
        motorConfig.r = values["r"].as<double>();
        motorConfig.R_M = values["R_M"].as<double>();
        motorConfig.G_r = values["G_r"].as<double>();
        motorConfig.E_e = values["E_e"].as<double>();
        
        if (! model->GetJoint(motorConfig.name) ) {
             ROS_ERROR_STREAM ("[ERROR HERI gazebo plugin]: motor " << motorConfig.name << 
                 " not found as a joint in the urdf. You should add it as virtual joint" << 
                 " so Gazebo can use PID on it");
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
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&HeriIIPlugin::OnUpdate, this));
    
}
    
void gazebo::HeriIIPlugin::OnUpdate()
{
    //here we move the finger joints, instead the motor is moved internally by gazebo because 
    // we have set pid controller for them
    
    for (auto it : motorsPositionCommand) {
        
        double theta_d = it.second;
        //0: heri motor has always one dof
        double theta = this->model->GetJoint(it.first)->GetAngle(0).Radian();
        
        MotorConfig motorConfig = motorsConfigs.at(it.first);

        double f_tendon = (current * torque_constant * gear_ratio * efficiency) / 0.008; //0.008 is the radius of the pulley (torque to force "conversion")
        std::vector<std::string> joints = moto_fingerJoints_map.at(it.first);

        double joint_DeltAngle[joints.size()];
        joint_DeltAngle[0] = JDA.DeltAngle_Join1(force);
        joint_DeltAngle[1] = JDA.DeltAngle_Join2(force);
        
        if (joints.size() > 2 ) { //no 3rd phalange for thumb
            joint_DeltAngle[2] = JDA.DeltAngle_Join3(force);
        }
        
        //Be sure joints in the map are in order
        int i = 0;
        for (auto joint : joints) {
            model->GetJoint(joint)->SetPosition(0, joint_DeltAngle[i] ); //0 is the joint axis
            i++;
        }
        
        //TODO if only simulation, we set the actual motor pos as the desired one
        // if real hardware is in use, we should take this info from the real hardware
        motors_position.at(it.first) = theta_d;
    }
}

void gazebo::HeriIIPlugin::motorCommandClbk(const sensor_msgs::JointStateConstPtr& msg) {
    
    for (int i = 0; i < msg->name.size(); i++) { 

        motors_position_command [msg->name.at(i)] = msg->position.at(i);
    }
    
}
