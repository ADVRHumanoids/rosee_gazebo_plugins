/** 
 * 
 *  std::string jstate_topic_name  = "joint_states";
    const int jstate_queue = 10;

    _joint_state_pub = _nh.advertise<sensor_msgs::JointState> ( jstate_topic_name, jstate_queue );
 * 
 */


#include <rosee_gazebo_plugins/rosee_plugin.h>

void gazebo::RoseePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Just output a message for now
    std::cout << "\nThe test_ee_plugin plugin is attach to model '" <<
        _model->GetName() << "'" << std::endl;
        
     // Safety check
    if (_model->GetJointCount() == 0)
    {
        std::cerr << "Invalid joint count, test_ee_plugin  not loaded\n";
        return;
    }

    // Store the model pointer for convenience.
    this->model = _model;
    //store all joints.. note that also fixed joints are stored
    joints = model->GetJoints();

    parseControllerConfig();

    setPIDs();


//     auto joint = _model->GetJoints()[1]; //bbase to left
//     if ( jointControllersConfigsMap.at(joint->GetName()).type.compare ("JointPositionController") == 0) {
//         this->model->GetJointController()->SetPositionTarget( joint->GetScopedName(), 0.5); 
//     } else if ( jointControllersConfigsMap.at(joint->GetName()).type.compare ("JointVelocityController") == 0) {
//         this->model->GetJointController()->SetVelocityTarget( joint->GetScopedName(), 0.1); 
//     }
//     
//     joint = _model->GetJoints()[2]; //left to leftip
//         if ( jointControllersConfigsMap.at(joint->GetName()).type.compare ("JointPositionController") == 0) {
//         this->model->GetJointController()->SetPositionTarget( joint->GetScopedName(), 0.5); 
//     } else if ( jointControllersConfigsMap.at(joint->GetName()).type.compare ("JointVelocityController") == 0) {
//         this->model->GetJointController()->SetVelocityTarget( joint->GetScopedName(), 0.1); 
//     }
// 
//     joint = _model->GetJoints()[3]; //base to right
//         if ( jointControllersConfigsMap.at(joint->GetName()).type.compare ("JointPositionController") == 0) {
//         this->model->GetJointController()->SetPositionTarget( joint->GetScopedName(), -0.5); 
//     } else if ( jointControllersConfigsMap.at(joint->GetName()).type.compare ("JointVelocityController") == 0) {
//         this->model->GetJointController()->SetVelocityTarget( joint->GetScopedName(), -0.1); 
//     }
//      
//     joint = _model->GetJoints()[4]; //righ to right tip
//         if ( jointControllersConfigsMap.at(joint->GetName()).type.compare ("JointPositionController") == 0) {
//         this->model->GetJointController()->SetPositionTarget( joint->GetScopedName(), -0.5); 
//     } else if ( jointControllersConfigsMap.at(joint->GetName()).type.compare ("JointVelocityController") == 0) {
//         this->model->GetJointController()->SetVelocityTarget( joint->GetScopedName(), -0.1); 
//     }
    

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "rosee_gazebo_plugin",
        ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("rosee_gazebo_plugin"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/ros_end_effector/joint_states",
        1,
        boost::bind(&RoseePlugin::jointStateClbk, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    
    //TODO relative scope?
    //TODO name of pub sub topics...
    rosPub = rosNode->advertise < sensor_msgs::JointState > ( "ros_end_effector/gazebo/joint_state", 1 ) ;


    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&RoseePlugin::QueueThread, this));
}


void gazebo::RoseePlugin::parseControllerConfig() {
    
    //TODO relative path
    //TODO check elsewhere if file exist or not
    //TODO USE THE ORIGINAL UTILS FROM ROSEE
    std::string dirPath = ROSEE::Utils::getPackagePath() + "configs/two_finger_control.yaml" ;
    std::ifstream ifile ( dirPath );
    if (! ifile) {
        ROS_ERROR_STREAM ("[ERROR gazebo plugin]: file " << dirPath << " not found. ");
            return;
    }
    
    YAML::Node node = YAML::LoadFile(dirPath);
    
    if ( ! node[model->GetName()] ) {
        ROS_ERROR_STREAM ("[ERROR gazebo plugin]: " << model->GetName() << " not found in the config file " << "two_finger_control.yaml" );
            return;
    }
    
    for ( auto controller: node[model->GetName()] ) {
        JointControllerConfig jcf;
        jcf.name = controller.first.as<std::string>();
        jcf.type = controller.second["type"].as<std::string>();
        jcf.jointName = controller.second["joint_name"].as<std::string>();
        jcf.p = controller.second["pid"]["p"].as<double>();
        jcf.i = controller.second["pid"]["i"].as<double>();
        jcf.d = controller.second["pid"]["d"].as<double>();
        
        if ( ! jointControllersConfigsMap.insert ( std::make_pair (jcf.jointName, jcf) ). second ) {
            ROS_ERROR_STREAM ( "[ERROR gazebo plugin]: " << " Multiple controllers for joint " << jcf.jointName << " are present in the config file " 
                << "two_finger_control.yaml" );
            return;
        }
    } 
}

//TODO, check the jointControllersConfigs and see which type want, now only position and vel
void gazebo::RoseePlugin::setPIDs() {
    
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
        } else {
            ROS_ERROR_STREAM ( "[ERROR gazebo plugin]:  Controller " << it->first << " is of type " << it->second.type
                << " which I don't recognize " );
        }
    }
}


void gazebo::RoseePlugin::jointStateClbk ( const sensor_msgs::JointStateConstPtr &_msg ) {
    jointStateMsg = *_msg;
}

void gazebo::RoseePlugin::QueueThread() {
    static const double timeout = 0.01;
    ros::Rate r(100); //100 Hz
    while (this->rosNode->ok()) {
        
        //TODO check the order of this three function
        
        pubJointState();
        
        //see if some messages for subs have arrived
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
        
        updatePIDfromParam();
        
        
        setReference();
        
        r.sleep();
    }
}

void gazebo::RoseePlugin::pubJointState ( ) {

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();

    for ( auto joint : joints ) {
        
        msg.name.push_back(joint->GetName()); //index 0 is the "right" axis
        msg.position.push_back(joint->GetAngle(0).Radian()); //index 0 is the "right" axis
        msg.velocity.push_back(joint->GetVelocity(0)); //index 0 is the "right" axis
        //WARNING getForce not yet implemented (in gazebo7...)
        msg.effort.push_back(joint->GetForce(0)); //index 0 is the "right" axis
        
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
            
        } else if ( it->second.type.compare ("JointVelocityController") == 0) {
            this->model->GetJointController()->SetVelocityTarget (
                scopedJointName, jointStateMsg.velocity.at ( i ) );
            
        } else if ( it->second.type.compare ("JointEffortController") == 0 ) {
            ROS_ERROR_STREAM ( "JointEffortController still not implememnted" );
            
        } else {
            ROS_ERROR_STREAM ("[ERROR gazebo plugin]:  Controller " << it->first << " is of type " << it->second.type
                << " which I don't recognize " );
        }
    }
}

//TODO update only if pids updated???
void gazebo::RoseePlugin::updatePIDfromParam() {
    
    for (auto & contrConf : jointControllersConfigsMap ) {
        
        double p, i, d;
        //TODO param name nicer
        std::string paramName = "/rosee_gazebo_plugin/" + model->GetName() + "/" + contrConf.second.name + "/pid" ;
        if ( rosNode->getParam ( (paramName + "/p"), p ) ) {
        } else {
            ROS_WARN_STREAM ( "[WARNING gazebo plugin] param " << ( paramName + "/p")  << " not loaded on parameter server" );
        }
        
        rosNode->getParam ( ( paramName + "/i"), i ) ;
        rosNode->getParam ( ( paramName + "/d"), d ) ;
        
        //update the map (for consistency only, not necessary in truth)
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
            ROS_ERROR_STREAM ( "JointEffortController still not implememnted" );
        } else {
            ROS_ERROR_STREAM ( "[ERROR gazebo plugin]:  Controller " << contrConf.first << " is of type " << contrConf.second.type
                << " which I don't recognize " );
        }
        
        
    }
    
    
}
