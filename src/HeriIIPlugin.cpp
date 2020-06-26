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

    for(YAML::const_iterator moto_it = node["Motor_actuation_info"].begin(); moto_it != node["Motor_actuation_info"].end(); ++moto_it) {
        
        std::string motor_name = moto_it->first.as<std::string>();
        auto values = moto_it->second;
        std::vector <std::string> associatedJoints;
        associatedJoints.push_back(values["joint_1"].as<std::string>());
        associatedJoints.push_back(values["joint_2"].as<std::string>());
        
        if (values["joint_3"].Type() != YAML::NodeType::value::Null ) { //no 3rd phalange for thumb
            associatedJoints.push_back(values["joint_3"].as<std::string>());
        }

        moto_fingerJoints_map[motor_name] = associatedJoints;
    }
    
    //TODO take this from some config file or launch file?
    std::string currentTopic = "/rosee_gazebo_plugins/motor_current_command";
    
    //TODO take this from conf file, and use different params for different motors?
    torque_constant = 0.0418;          //Torque contant of motor. DCX22S GB KL 48V.
    gear_ratio = 139;                   //GPX22HP 138.
    efficiency = 0.85;                 //The maximum efficieny of gear box is 0.88.
    
    rosSub = rosNode->subscribe(currentTopic, 1, &gazebo::HeriIIPlugin::currentCommandClbk, this);
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&HeriIIPlugin::OnUpdate, this));
    
}
    
void gazebo::HeriIIPlugin::OnUpdate()
{
    
    for (auto it : moto_current_map) {
        
        double current = it.second;

        double force = (current * torque_constant * gear_ratio * efficiency) / 0.008; //0.008 is the radius of the pulley (torque to force "conversion")
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
    }
}

void gazebo::HeriIIPlugin::currentCommandClbk(const rosee_msg::MotorCurrentConstPtr& msg) {
    
    for (int i = 0; i < msg->motor_name.size(); i++) { 

        moto_current_map [msg->motor_name.at(i)] = msg->current.at(i);
    }
    
}
