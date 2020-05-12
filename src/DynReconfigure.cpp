/*
 * Copyright 2020 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rosee_gazebo_plugins/DynReconfigure.h>

int main(int argc, char **argv) { 
    
    ros::init(argc, argv, "rosee_gazebo_plugin_DynReconfigure");
    ros::NodeHandle rosNode("rosee_gazebo_plugin_DynReconfigure");

    if (argc < 2) {
        ROS_ERROR_STREAM ( "please insert the robotName as argument" ) ;
        return -1; 
    }    
    //get command line arguments - the name of the robot
    std::string robotName = argv[1];
    
    std::vector <std::string> controllersName;
    if (! ROSEE::DynReconfigure::parseControllersNames ( &rosNode, controllersName )) {
        return -1;
    }
    
    //we have to "store" all the vectors to not make the servers go out of scope
    std::vector< std::shared_ptr <dynamic_reconfigure::Server<rosee_gazebo_plugins::pidConfig> > > drVector;

    for (auto controller : controllersName) {
        
        ros::NodeHandle nh ("/rosee_gazebo_plugins/rosee_gazebo_plugins_args/" + controller + "/pid") ;
        std::shared_ptr <dynamic_reconfigure::Server<rosee_gazebo_plugins::pidConfig> > dr_srv_ptr = 
           std::make_shared <dynamic_reconfigure::Server<rosee_gazebo_plugins::pidConfig>> ( nh ) ;
        
        dynamic_reconfigure::Server<rosee_gazebo_plugins::pidConfig>::CallbackType cb;
        cb = boost::bind(&ROSEE::DynReconfigure::pidCfgClbk, _1, _2, 
                         "/rosee_gazebo_plugins/rosee_gazebo_plugins_args/" + controller + "/pid");
        dr_srv_ptr->setCallback(cb);
        drVector.push_back ( dr_srv_ptr );

    }
       
    ros::spin();
    return 0;
    
}

bool ROSEE::DynReconfigure::parseControllersNames (ros::NodeHandle* rosNode, 
                                                   std::vector<std::string> &controllersName) {
    
    std::string configFileName;
    
    if (! rosNode->getParam("/ros_ee_config_path", configFileName) ) {
        ROS_ERROR_STREAM ("[ERROR Dynamic Reconfigure]: parameter '/ros_ee_config_path'" <<
            " not loaded on parameter server");
        return false;
    }
    
    YAML::Node node = YAML::LoadFile(configFileName);
    
    if ( ! node["rosee_gazebo_plugins_args"] ) {
        ROS_ERROR_STREAM ("[ERROR Dynamic Reconfigure]: Not found the node 'rosee_gazebo_plugins_args' not found in the config file '" << configFileName << "' , or the file is missing" );
            return false;
    }
    
    for (const auto & controller : node["rosee_gazebo_plugins_args"] ) {
        
        controllersName.push_back ( controller.first.as<std::string>() ) ;
    }
    
    return true;
}


void ROSEE::DynReconfigure::pidCfgClbk ( rosee_gazebo_plugins::pidConfig &config, uint32_t level, std::string paramName) {
  ROS_INFO_STREAM("Reconfigure Request: " << "for " << paramName << " : " <<
            "p=" << config.p << "   " << 
            "i=" << config.i << "   " <<
            "d=" << config.d);
}
