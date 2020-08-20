#ifndef _HERI_II_PLUGIN_HH_
#define _HERI_II_PLUGIN_HH_

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>

#include <rosee_gazebo_plugins/JointDeltAngle.h>

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
        
        struct MotorConfig {
            const std::string name;
            const std::vector<std::string> linked_joints; //name of joints of the finger
            double p; //pid gains
            double i;
            double d;
            double K_t; // Torque coefficent of the motor [Nm/A]
            double r;   // Radius of the pulley [m]
            double R_M; // Eletrical terminal resistance of the motor [ohm]
            double G_r;  // Ratio of the gearbox of the motor [adimensional]
            double E_e; // Efficency of gearhead, 0.9 is the max efficency of the motor [adimensional]
        } ;
        
        std::map <std::string, MotorConfig> motorsConfigs;
        
        std::map <std::string, double> motorsPositionCommand;
        
        // Pointer to the model
        physics::ModelPtr model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        
        /// \brief A node use for ROS transport
        std::unique_ptr<ros::NodeHandle> rosNode;
        
        /// \brief A ROS subscriber
        ros::Subscriber rosSub;
        ros::Publisher rosPub;
        
        JointDeltAngle JDA;
        
        bool newCommand;
        void motorCommandClbk(const sensor_msgs::JointStateConstPtr& msg);
        
        void pubJointStates();


        
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(HeriIIPlugin)
}

#endif // _HERI_II_PLUGIN_HH_
