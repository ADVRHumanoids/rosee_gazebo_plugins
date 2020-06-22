#ifndef _HERI_II_PLUGIN_HH_
#define _HERI_II_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <rosee_gazebo_plugins/JointDeltAngle.h>
#include <rosee_gazebo_plugins/JointMechanism.h>

namespace gazebo
{
    /**
    * @brief This class is a gazebo plugin which job is to send command to simulated robot in gazebo.
    * It is similar to gazebo_ros_control
    * 
    * @todo Check if necessary to update always the pid gains
    * @todo Check namespaces for published topics and used ros parameters
    * 
    */
    class HeriIIPlugin : public ModelPlugin {
    
    public: 
        //HeriIIPlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        
        // Called by the world update start event
        void OnUpdate();

     
    private:
        
        // Pointer to the model
        physics::ModelPtr model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        
        JointDeltAngle JDA;
        
        std::map<std::string, double> moto_current_map;
        std::map<std::string, std::vector<std::string>> moto_fingerJoints_map;
        
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(HeriIIPlugin)
}

#endif // _HERI_II_PLUGIN_HH_