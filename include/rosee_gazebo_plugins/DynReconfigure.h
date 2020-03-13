/*
 * Copyright 2020 <Davide Torielli> <toridebraus@gmail.com>
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

#ifndef ROSEE_DYNRECONFIGURE_H
#define ROSEE_DYNRECONFIGURE_H

#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>

// Auto-generated from cfg/ directory.
//TODO check why this is generated inside this folder and not in the build folder
#include "../../cfg/cpp/rosee_gazebo_plugins/pidConfig.h"

#include <yaml-cpp/yaml.h>
#include <ROSEndEffector/Utils.h>

/**
 * @brief main to handle the ros node dynamic_reconfigure  
 * @todo check why config file is generated inside src/cfg/cpp/rosee_gazebo_plugins folder and not in the build folder
 * @todo send an updated pid so the rosee_plugin updated the pid only when requested?
 */
int main(int argc, char **argv) ;

namespace ROSEE {
namespace DynReconfigure {

    /**
     * @brief callback for when pids are updated for some controller. It simply print some info messages
     * because the param on the param server are updated by the dynamic_reconfigure server
     * @param &config the auto generated (from .cfg) config file
     * @param level
     * @paramName the name of the param to be updated, used here to print info messages about the update
     */
    void pidCfgClbk ( rosee_gazebo_plugins::pidConfig &config, uint32_t level, std::string paramName) ;
    
    /**
     * @brief parse the controller names from the yaml file configs/ROBOTNAME_control.yaml (the same file used by
     * \ref rosee_plugin.cpp ). Names are necessary to "point" to the right param of ros parameter server
     * 
     * @param robotName (IN) the name of the robot, which is part of filename of yaml file
     * @param &controllersName (OUT) vector containing all the controllers name taken from yaml config file
     */
    bool parseControllersName(std::string robotName, std::vector<std::string> &controllersName) ;

}
}

#endif // ROSEE_DYNRECONFIGURE_H
