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

#ifndef ROSEE_DYNRECONFIGURE_H
#define ROSEE_DYNRECONFIGURE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
//TODO check why this is generated inside this folder and not in the build folder
#include "../../cfg/cpp/rosee_gazebo_plugins/pidConfig.h"

#include <yaml-cpp/yaml.h>
#include <rosee_gazebo_plugins/Utils.h>



namespace ROSEE {
namespace DynReconfigure {

void pid_cfg_clbk ( rosee_gazebo_plugins::pidConfig &config, uint32_t level, std::string paramName) ;
void parseControllerConfig (std::string &handName, std::vector<std::string> &controllersName) ;





}
}

#endif // ROSEE_DYNRECONFIGURE_H
