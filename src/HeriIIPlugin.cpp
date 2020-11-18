#include <rosee_gazebo_plugins/HeriIIPlugin.h>

void gazebo::HeriIIPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    std::cout << "The HeriII plugin is attach to model '" <<
        _model->GetName() << "'" << std::endl;
        
     // Safety check
    if (_model->GetJointCount() == 0)
    {
        std::cout << "[ERROR gazebo plugin] Invalid joint count, it is zero! " << std::endl;
        return;
    }

    // Store the model pointer for convenience.
    this->model = _model;
    //store all joints.. note that also fixed joints are stored
    joints = model->GetJoints();
    
    // Initialize ros, if it has not already been initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "heriII_gazebo_plugin", ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. 
    this->rosNode.reset(new ros::NodeHandle("heriII_gazebo_plugins"));

    if (! parseMotorsConfig() ) {
        return;
    }

    if (! setPIDs() ) {
        return;
    }




    // Create a named topic, and subscribe to it.
    //TODO take namespace from somewhere
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/dummyHal/joint_command",
        1,
        boost::bind(&HeriIIPlugin::jointStateClbk, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    
    rosPub = rosNode->advertise < sensor_msgs::JointState > ( "/dummyHal/joint_states", 1 ) ;

    // Spin up the queue helper thread.
    double rate = 0.0;
    if (! rosNode->getParam("/rate", rate)){
        ROS_INFO_STREAM ("Ros parameter for rate not found, I'm setting the default rate of 100 Hz");
        rate = 100.0;
    }

    this->rosQueueThread = std::thread(std::bind(&HeriIIPlugin::QueueThread, this, rate));


}


void gazebo::HeriIIPlugin::QueueThread(double rate) {
    ros::Rate r(rate); 
    while (this->rosNode->ok()) {
        
        //TODO check the order of this three function

        pubJointState();
        // see if some messages for subs have arrived
        this->rosQueue.callAvailable(ros::WallDuration());

        updatePIDfromParam();
        
        setReference();
                
        r.sleep();
    }
}


bool gazebo::HeriIIPlugin::parseMotorsConfig() {
    
    std::string configFileName;
    
    if (! rosNode->getParam("/ros_ee_config_path", configFileName) ) {
        ROS_ERROR_STREAM ("[ERROR gazebo plugin]: parameter '/ros_ee_config_path'" <<
            " not loaded on parameter server");
        return false;
    }
    
    YAML::Node node = YAML::LoadFile(configFileName);
    
    if ( ! node["Motor_actuation_info"] ) {
        ROS_ERROR_STREAM ("[ERROR heriII gazebo plugin]: Not found the node 'Motor_actuation_info' not found in the config file '" << configFileName << "' , or the file is missing" );
            return false;
    }
    
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
        
        
        if (! model->GetJoint(motorConfig.motorName) ) {
            ROS_ERROR_STREAM ("[ERROR HERI gazebo plugin]: motor '" << motorConfig.motorName << 
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
            return false ;
        }
        
        //gazebo pid to move the motors
        //this->model->GetJointController()->SetPositionPID(
            //    model->GetJoint(motorConfig.name)->GetScopedName(), common::PID(motorConfig.p, motorConfig.d, motorConfig.d));
        
        motorsConfigMap.insert( std::make_pair ( motorConfig.motorName , motorConfig));
        
        //Set the pid on the param server, this is needed by dynamic reconfigure server
 
        std::string rootNameParam = "/rosee_gazebo_plugins/params/" ;

        rosNode->setParam ( rootNameParam + motorConfig.motorName + "/pid/p", motorConfig.p );
        rosNode->setParam ( rootNameParam + motorConfig.motorName + "/pid/i", motorConfig.i );
        rosNode->setParam ( rootNameParam + motorConfig.motorName + "/pid/d", motorConfig.d );

    }
    
    return true;
}

bool gazebo::HeriIIPlugin::setPIDs() {
    
    for (auto joint : joints ) {
        
        auto it = motorsConfigMap.find (joint->GetName());
        if ( it == motorsConfigMap.end() ) {
            continue;
        }

        // Apply the controller to the joint. We always apply a jointpositioncontroller
        // because also real heri can be controlled only in position
        
        this->model->GetJointController()->SetPositionPID(
            joint->GetScopedName(), common::PID(it->second.p, it->second.d, it->second.i));
            
    }
    return true;
}


void gazebo::HeriIIPlugin::jointStateClbk ( const sensor_msgs::JointStateConstPtr &_msg ) {
    jointStateMsg = *_msg;
}

void gazebo::HeriIIPlugin::pubJointState ( ) {

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();

    for ( auto joint : joints ) {
        //TODO also mimic are returned... leave as it is?
        //type of joint is a sum: so fixed is joint+fixed_joint (0x4000 + 0x40 = 0x4040)
        if (joint->GetType() != 
                (gazebo::physics::Entity::JOINT + gazebo::physics::Entity::FIXED_JOINT) ) {
            
            msg.name.push_back(joint->GetName());
            msg.position.push_back(joint->GetAngle(0).Radian()); //index 0 is the "right" axis
            msg.velocity.push_back(joint->GetVelocity(0)); //index 0 is the "right" axis
            //WARNING getForce not yet implemented (in gazebo7 is an empty function...)
            msg.effort.push_back(joint->GetForce(0)); //index 0 is the "right" axis
        }
    }
    
    rosPub.publish(msg);
}

void gazebo::HeriIIPlugin::setReference (  )
{        
    
    for (int i = 0; i < jointStateMsg.name.size(); i++) { 
        
        std::string motorName = jointStateMsg.name.at(i);
        std::string scopedMotorName = model->GetName() + "::" + motorName; //necessary for Set of gazebo
        
        auto it = motorsConfigMap.find ( motorName );
        if ( it == motorsConfigMap.end() ) {
            continue;
        }

        // Apply the reference basing on the type of controller 
        this->model->GetJointController()->SetPositionTarget (
            scopedMotorName, jointStateMsg.position.at ( i ) );
        
        moveCoupledJoints(motorName, jointStateMsg.position.at ( i ) );

    }
}

void gazebo::HeriIIPlugin::moveCoupledJoints(std::string motorName, double motorRef) {
    
    //TODO : retrieve pid and associated links from motorconfig of motorName
    //then move with joindDeltAngle class
    
    
}


//TODO update only if pids updated???
void gazebo::HeriIIPlugin::updatePIDfromParam() {
    
    for (auto & contrConf : motorsConfigMap ) {
        
        double p, i, d;
        std::string paramName = "/rosee_gazebo_plugins/params/" + contrConf.second.motorName + "/pid" ;
        
        if ( ! rosNode->getParam ( (paramName + "/p"), p ) ) {
        
            ROS_WARN_STREAM ( "[WARNING gazebo plugin] param " << ( paramName + "/p")  << " not loaded on parameter server" );
        }
        
        if ( ! rosNode->getParam ( (paramName + "/i"), i ) ) {
      
            ROS_WARN_STREAM ( "[WARNING gazebo plugin] param " << ( paramName + "/i")  << " not loaded on parameter server" );
        }
        
        if ( ! rosNode->getParam ( (paramName + "/d"), d ) ) {

            ROS_WARN_STREAM ( "[WARNING gazebo plugin] param " << ( paramName + "/d")  << " not loaded on parameter server" );
        }
        
        //update the map (for consistency only, not necessary in truth because pids of map are only
        // checked once in setPID function)
        contrConf.second.p = p ;
        contrConf.second.i = i ;
        contrConf.second.d = d ;
        
        model->GetJointController()->SetPositionPID(
            model->GetJoint(contrConf.second.motorName) ->GetScopedName(), common::PID(p, i, d));
    }
}
