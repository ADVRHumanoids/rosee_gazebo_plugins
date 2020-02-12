#ifndef _ROSEE_PLUGIN_HH_
#define _ROSEE_PLUGIN_HH_

#include <thread>
#include <ros/ros.h>
#include <ros/console.h>

#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>

#include <gazebo/gazebo_client.hh>
#include <gazebo/physics/physics.hh>

#include <yaml-cpp/yaml.h>
#include <rosee_gazebo_plugins/Utils.h>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class RoseePlugin : public ModelPlugin
  {
    
    public: 
        /// \brief Constructor
        RoseePlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        

        void pubJointState();
        
        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
        void jointStateClbk (const sensor_msgs::JointStateConstPtr&_msg);

     
    private:
        
        /// \brief ROS helper function that processes messages
        void QueueThread();
        
        void parseControllerConfig();
        void setPIDs();
        void setReference ( );
        void updatePIDfromParam();




        /// \brief Pointer to the model.
        physics::ModelPtr model;

        /// \brief Pointer to the joint.
        std::vector <physics::JointPtr> joints;
        sensor_msgs::JointState jointStateMsg;
        
        /// \brief A node use for ROS transport
        std::unique_ptr<ros::NodeHandle> rosNode;
        
        /// \brief A ROS subscriber
        ros::Subscriber rosSub;
        ros::Publisher rosPub;
        
        /// \brief A ROS callbackqueue that helps process messages
        ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        std::thread rosQueueThread;
        
        struct JointControllerConfig {
            std::string name;
            std::string type;
            std::string jointName;
            double p;
            double i;
            double d;
        } ;
        
        std::map <std::string, JointControllerConfig> jointControllersConfigsMap;
        
        
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(RoseePlugin)
}



#endif // _ROSEE_PLUGIN_HH_
