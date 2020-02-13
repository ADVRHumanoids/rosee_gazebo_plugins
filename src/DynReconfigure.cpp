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
    
    if (argc < 2) {
        ROS_ERROR_STREAM ( "please insert the robotName as argument" ) ;
        return -1; 
    }    
    //get command line arguments - the name of the robot
    std::string robotName = argv[1];
    
    std::vector <std::string> controllersName;
    if (! ROSEE::DynReconfigure::parseControllerConfig (robotName, controllersName )) {
        return -1;
    }
    
    //we have to "store" all the vectors to not make them go out of scope
    std::vector< std::shared_ptr <dynamic_reconfigure::Server<rosee_gazebo_plugins::pidConfig> > > drVector;

    for (auto controller : controllersName) {
        
        ros::NodeHandle nh ("/rosee_gazebo_plugin/" + robotName + "/" + controller + "/pid") ;
        std::shared_ptr <dynamic_reconfigure::Server<rosee_gazebo_plugins::pidConfig> > dr_srv_ptr = 
           std::make_shared <dynamic_reconfigure::Server<rosee_gazebo_plugins::pidConfig>> ( nh ) ;
        
        dynamic_reconfigure::Server<rosee_gazebo_plugins::pidConfig>::CallbackType cb;
        cb = boost::bind(&ROSEE::DynReconfigure::pid_cfg_clbk, _1, _2, "/rosee_gazebo_plugin/" + robotName + "/" + controller + "/pid");
        dr_srv_ptr->setCallback(cb);
        drVector.push_back ( dr_srv_ptr );
        
        ros::spinOnce();
    }
       
    ros::spin();
    return 0;
    
}

/* Here we parse only the hand name and controller name, to know the name of the parameter loaded in the launchfile */
bool ROSEE::DynReconfigure::parseControllerConfig (std::string robotName, std::vector<std::string> &controllersName) {
    
    //TODO relative path
    //TODO check elsewhere if file exist or not
    //TODO USE THE ORIGINAL UTILS FROM ROSEE
    std::string dirPath = ROSEE::Utils::getPackagePath() + "configs/" + robotName + "_control.yaml" ;
    std::ifstream ifile ( dirPath );
    if (! ifile) {
        ROS_ERROR_STREAM ( "[ERROR gazebo plugin]: file " << dirPath << " not found. " );
            return false;
    }
    
    YAML::Node node = YAML::LoadFile(dirPath);
    // this for is necessary to take the first line of yaml file (the robot name). In truth there is only a name
    // so this loop is done only once
    for (const auto& kv : node) {
        
        if (robotName.compare (kv.first.as<std::string>()) != 0 ) {
            ROS_ERROR_STREAM ( "'" << robotName << "' taken as argument, does not match the robot name found in" << 
                "configs/" + robotName + "_control.yaml that is: '" <<  kv.first.as<std::string>() << "'");
        }
        for (const auto & controller: kv.second ) {
            controllersName.push_back ( controller.first.as<std::string>() ) ;
        }
    }
    return true;
}

//TODO un updated_pid così il rosee_plugin aggiorna i pid solo se updated??
void ROSEE::DynReconfigure::pid_cfg_clbk( rosee_gazebo_plugins::pidConfig &config, uint32_t level, std::string paramName) {
  ROS_INFO_STREAM("Reconfigure Request: " << "for " << paramName << " : " <<
            "p=" << config.p << "   " << 
            "i=" << config.i << "   " <<
            "d=" << config.d);
}

