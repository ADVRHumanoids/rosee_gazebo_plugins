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
//to find relative path for the config files
#include <boost/filesystem/path.hpp>
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
    class RoseePlugin : public ModelPlugin {
    
    public: 
        RoseePlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        


     
    private:
        
        /// \brief ROS helper function that run all the periodic functions
        /// @param rate the rate (in Hz) of the running sequence
        void QueueThread(double rate);
        
        /**
         * @brief Parse the yaml file where infos about how to control each joint are present
         * @return false if some errors happened
         * @warning the yaml file must be located in configs folder and be named "ROBOTNAME_control.yaml"
         */
        bool parseControllerConfig();
        
        /**
         * @brief Set the pid values, which are taken from the controller yaml config file (parsed in \ref parseControllerConfig )
         * @return false if some errors happened
         */
        bool setPIDs();
        
        /**
         * @brief publish Joint states derived from gazebo simulation. 
         * @warning all the joint states are published, included fixed and mimic (i.e. not actuated). 
         * With gazebo, it would be easy to not consider the fixed but it is more complicated to exclude the not actuated
         */
        void pubJointState();
        
        /**
         * @brief update the references, that are taken from subscribed topic. Depending on the type of control,
         * (defined in the yaml file _control, parsed inf \ref parseControllerConfig, the references can be positions or velocities, for now
         */
        void setReference ( );
        
        /**
         * @brief PID gains for pos and vel (for now) can be modifed during the execution, updating the parameters on ros server.
         * To update them, is useful to use the \ref DynReconfigure node
         */
        void updatePIDfromParam();
        
        /// \brief callback to handle an incoming jointstate that is the control reference.
        /// \param[in] _msg A JointState message taken as control reference
        void jointStateClbk (const sensor_msgs::JointStateConstPtr&_msg);

        /// \brief Pointer to the model.
        physics::ModelPtr model;

        /// \brief vector of pointers to all the model joints (included fixed and not actuated).
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
        
        /**
         * @brief struct to store the infos about controller for a joint.
         */
        struct JointControllerConfig {
            std::string name;
            std::string type;
            std::string jointName;
            double p;
            double i;
            double d;
        } ;
        
        /**
         * @brief the map with as key the joint name and as value a \ref JointControllerConfig which contains controller infos
         */
        std::map <std::string, JointControllerConfig> jointControllersConfigsMap;
        
        
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(RoseePlugin)
}

#endif // _ROSEE_PLUGIN_HH_
