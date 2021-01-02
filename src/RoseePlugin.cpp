#include <rosee_gazebo_plugins/RoseePlugin.h>

void gazebo::RoseePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    std::cout << "The RoseePlugin plugin is attach to model '" <<
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
        ros::init(argc, argv, "rosee_gazebo_plugin", ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. 
    this->rosNode.reset(new ros::NodeHandle("rosee_gazebo_plugins"));

    if (! parseControllerConfig() ) {
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
        boost::bind(&RoseePlugin::jointStateClbk, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    
    rosPub = rosNode->advertise < sensor_msgs::JointState > ( "/dummyHal/joint_states", 1 ) ;

    // Spin up the queue helper thread.
    double rate = 0.0;
    if (! rosNode->getParam("/rate", rate)){
        ROS_INFO_STREAM ("Ros parameter for rate not found, I'm setting the default rate of 100 Hz");
        rate = 100.0;
    }

    this->rosQueueThread = std::thread(std::bind(&RoseePlugin::QueueThread, this, rate));


}


void gazebo::RoseePlugin::QueueThread(double rate) {
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


bool gazebo::RoseePlugin::parseControllerConfig() {
    
    std::string configFileName;
    
    if (! rosNode->getParam("/ros_ee_config_path", configFileName) ) {
        ROS_ERROR_STREAM ("[ERROR gazebo plugin]: parameter '/ros_ee_config_path'" <<
            " not loaded on parameter server");
        return false;
    }
    
    YAML::Node node = YAML::LoadFile(configFileName);
    
    if ( ! node["rosee_gazebo_plugins_args"] ) {
        ROS_ERROR_STREAM ("[ERROR gazebo plugin]: Not found the node 'rosee_gazebo_plugins_args' not found in the config file '" << configFileName << "' , or the file is missing" );
            return false;
    }
    
    for ( auto controller: node["rosee_gazebo_plugins_args"] ) {
        JointControllerConfig jcf;
        jcf.name = controller.first.as<std::string>();
        jcf.type = controller.second["type"].as<std::string>();
        jcf.jointName = controller.second["joint_name"].as<std::string>();
        jcf.p = controller.second["pid"]["p"].as<double>();
        jcf.i = controller.second["pid"]["i"].as<double>();
        jcf.d = controller.second["pid"]["d"].as<double>();
        
        if ( ! jointControllersConfigsMap.insert ( std::make_pair (jcf.jointName, jcf) ). second ) {
            ROS_ERROR_STREAM ( "[ERROR gazebo plugin]: " << " Multiple controllers for joint " << jcf.jointName 
            << " are present in the config file " <<  model->GetName() + "_control.yaml" );
            return false;
        }

        
        //Set the pid on the param server, this is needed by dynamic reconfigure server
 
        std::string rootNameParam = "/rosee_gazebo_plugins/params/" ;

        rosNode->setParam ( rootNameParam + jcf.name + "/pid/p", jcf.p );
        rosNode->setParam ( rootNameParam + jcf.name + "/pid/i", jcf.i );
        rosNode->setParam ( rootNameParam + jcf.name + "/pid/d", jcf.d );
    }
    
    return true;
}

bool gazebo::RoseePlugin::setPIDs() {
    
    for (auto joint : joints ) {
        
        auto it = jointControllersConfigsMap.find (joint->GetName());
        if ( it == jointControllersConfigsMap.end() ) {
            continue;
        }

        // Apply the controller to the joint.
        if ( it->second.type.compare ("JointPositionController") == 0) {
            this->model->GetJointController()->SetPositionPID(
                joint->GetScopedName(), common::PID(it->second.p, it->second.d, it->second.i));
            
        } else if ( it->second.type.compare ("JointVelocityController") == 0) {
            this->model->GetJointController()->SetVelocityPID(
                joint->GetScopedName(), common::PID(it->second.p, it->second.d, it->second.i));
        } 
        else if ( it->second.type.compare ("JointEffortController") == 0 ) {
            ROS_ERROR_STREAM ( "JointEffortController still not implememnted" );
            return false;
        } else {
            ROS_ERROR_STREAM ( "[ERROR gazebo plugin]:  Controller " << it->first << " is of type " << it->second.type
                << " which I don't recognize " );
            return false;
        }
    }
    return true;
}


void gazebo::RoseePlugin::jointStateClbk ( const sensor_msgs::JointStateConstPtr &_msg ) {
    jointStateMsg = *_msg;
}

void gazebo::RoseePlugin::pubJointState ( ) {

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();

    for ( auto joint : joints ) {
        //TODO also mimic are returned... leave as it is?
        //type of joint is a sum: so fixed is joint+fixed_joint (0x4000 + 0x40 = 0x4040)
        if (joint->GetType() != 
                (gazebo::physics::Entity::JOINT + gazebo::physics::Entity::FIXED_JOINT) ) {
            
            msg.name.push_back(joint->GetName());
#if GAZEBO_MAJOR_VERSION >= 8
            msg.position.push_back(joint->Position(0)); //index 0 is the "right" axis
#else
            msg.position.push_back(joint->GetAngle(0).Radian()); //index 0 is the "right" axis
#endif
            msg.velocity.push_back(joint->GetVelocity(0)); //index 0 is the "right" axis
            //WARNING getForce not yet implemented (in gazebo7 is an empty function...)
            msg.effort.push_back(joint->GetForce(0)); //index 0 is the "right" axis
        }
    }
    
    rosPub.publish(msg);
}

void gazebo::RoseePlugin::setReference (  )
{        
    
    for (int i = 0; i < jointStateMsg.name.size(); i++) { 
        
        std::string jointName = jointStateMsg.name.at(i);
        std::string scopedJointName = model->GetName() + "::" + jointName; //necessary for Set of gazebo
        
        auto it = jointControllersConfigsMap.find ( jointName );
        if ( it == jointControllersConfigsMap.end() ) {
            continue;
        }

        // Apply the reference basing on the type of controller 
        if ( it->second.type.compare ("JointPositionController") == 0) {
            this->model->GetJointController()->SetPositionTarget (
                scopedJointName, jointStateMsg.position.at ( i ) );
            
            
            //test TODO remove this
            //auto pidForJoint = this->model->GetJointController()-> GetPositionPIDs().at(scopedJointName);
            //double ep, ei, ed;
            //pidForJoint.GetErrors(ep, ei, ed);
            
            //ROS_WARN_STREAM("PID " << jointName << "\n  target: " <<  jointStateMsg.position.at ( i )
            //    << "\n joint pos " << this->model->GetJoint(scopedJointName)->GetAngle(0).Radian()
            //    << "\n pid cmd: " << pidForJoint.GetCmd()
            //   << "\n pid errors (p i d) " << ep << " " << ei << " " << ed 
            //);
            
        } else if ( it->second.type.compare ("JointVelocityController") == 0) {
            this->model->GetJointController()->SetVelocityTarget (
                scopedJointName, jointStateMsg.velocity.at ( i ) );
            
        } else if ( it->second.type.compare ("JointEffortController") == 0 ) {
            // should have checked this error before in setPIDs
            ROS_ERROR_STREAM ( "JointEffortController still not implememnted" );
            return;
            
        } else {
            // should have checked this error before in setPIDs
            ROS_ERROR_STREAM ("[ERROR gazebo plugin]:  Controller " << it->first << " is of type " << it->second.type
                << " which I don't recognize " );
            return;

        }
    }
}

//TODO update only if pids updated???
void gazebo::RoseePlugin::updatePIDfromParam() {
    
    for (auto & contrConf : jointControllersConfigsMap ) {
        
        double p, i, d;
        std::string paramName = "/rosee_gazebo_plugins/params/" + contrConf.second.name + "/pid" ;
        
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
        
        if ( contrConf.second.type.compare ("JointPositionController") == 0) {
            model->GetJointController()->SetPositionPID(
                model->GetJoint(contrConf.second.jointName) ->GetScopedName(), common::PID(p, i, d));
            
        } else if ( contrConf.second.type.compare ("JointVelocityController") == 0) {
            model->GetJointController()->SetVelocityPID(
                model->GetJoint(contrConf.second.jointName) ->GetScopedName(), common::PID(p, i, d));
        } 
        else if ( contrConf.second.type.compare ("JointEffortController") == 0 ) {
            // should have checked this error before in parseControllerConfig
            ROS_ERROR_STREAM ( "JointEffortController still not implememnted" );
            return;
        } else {
            // should have checked this error before in parseControllerConfig
            ROS_ERROR_STREAM ( "[ERROR gazebo plugin]:  Controller " << contrConf.first << " is of type " << contrConf.second.type
                << " which I don't recognize " );
            return;
        }        
    }
}
