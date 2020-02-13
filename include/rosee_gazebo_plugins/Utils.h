/*
 * Copyright (C) 2019 IIT-HHCM
 * Author: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef __ROSEE_UTILS__
#define __ROSEE_UTILS__

#include <cmath>
#include <memory>

//to find relative path for the config files and create directories
#include <boost/filesystem.hpp>

namespace ROSEE
{

namespace Utils
{
    
    
static std::string getPackagePath() {
    
    boost::filesystem::path path(__FILE__);
    path.remove_filename();
    return path.string() + "/../../";
}





}

}

#endif // __ROSEE_UTILS__
