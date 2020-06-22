#include <rosee_gazebo_plugins/HeriIIPlugin.h>

void gazebo::HeriIIPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
 
    // Store the pointer to the model
    this->model = _parent;
    
    // Error message if the model couldn't be found
    if (!model) {
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
    
    for (auto it : moto_current_map) {
        
        double current = it.second;
        
        double Joint1_DeltAngle = JDA.DeltAngle_Join1(current);
        double Joint2_DeltAngle = JDA.DeltAngle_Join2(current);
        double Joint3_DeltAngle = JDA.DeltAngle_Join3(current);
    
        std::vectors<std::string> joints = moto_fingerJoints_map.at(it.first);
        
        //HACK be sure joints are in order
        for (auto joint : joints) {
            model->GetJoint(joint)->SetPosition(Joint1_DeltAngle);
            //TODO
        }
    }
}

//TODO
/**
 
 Get current from ros param, current will be a message with joint name (or motor id) and a
 double value. store the currents when they arrive in ros clbk. 
 in onupdate, update the joint position according to the current stored.
 
 
 */ 
