#include <rosee_gazebo_plugins/HeriIIPlugin.h>

void gazebo::HeriIIPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
 
    // Store the pointer to the model
    this->model = _parent;
    
    // Error message if the model couldn't be found
    if (!model_) {
        std::cout << "Model is NULL! HeriIIPlugin could not be loaded.");
        return;
    
    }
    
    
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&HeriIIPlugin::OnUpdate, this, _1));

}
    
void gazebo::HeriIIPlugin::OnUpdate()
{
    model->GetJoint("LFB1__LFP2_1")->SetPosition()
    ...
}

//TODO
/**
 
 Get current from ros param, current will be a message with joint name (or motor id) and a
 double value. store the currents when they arrive in ros clbk. 
 in onupdate, update the joint position according to the current stored.
 
 
 */ 
