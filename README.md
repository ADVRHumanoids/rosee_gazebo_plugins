# rosee_gazebo_plugins
#### Utility package to simulate robot in gazebo for [ROSEE](https://github.com/ADVRHumanoids/ROSEndEffector) project
*****
It is very similar to the "official" ROS package for control : [gazebo_ros_control](http://gazebosim.org/tutorials/?tut=ros_control)

It includes a gazebo plugin to take commands (*references*) from **/ros_end_effector/joint_commands** topic and use to move the simulated robot in gazebo. The same plugin also publish the "real" joint States (derived by the gazebo simulation) in the topic **/rosee_gazebo_plugin/joint_states**.

**Take care:** In */ros_end_effector/joint_commands* topic they are sent commands for only some joints (and never for not actuated ones); while in */rosee_gazebo_plugin/joint_states* there are state for ALL non-fixed joints, included not actuated (i.e. mimic). This may change in future, but for now I leave as it is because returning also mimic joint is not so a problem.


At the moment each joint can be commanded in position or in velocity, using the *SetPositionTarget()* and *SetVelocityTarget()* of gazebo. But please note that, for now, reference from rosee are only in position.
Gains (pid) are settable in a config file and during execution with dynamic reconfigurator (see later for istructions). This one is the *rosee_gazebo_plugin_DynReconfigure* which create  *dynamic_reconfigure* servers to give the possibility to change the PID params during the simulation.

#### Mimic Joints
Thanks to the [mimic_joint_plugin](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins) (and to [mimic_joint_gazebo_tutorial](https://github.com/mintar/mimic_joint_gazebo_tutorial) to understand how to use it), now also mimic joints work correctly. The argument set_pid of this plugin should be set to false, because otherwise a pid controller is used, that is the "default" gazebo one (see plugin page for more info). And also when mimicking, we want to reply exactly the angle of the other joint (or not?). See above how to deal with this plugin if your urdf model has some mimic joint

## How to Run
You should follow main RosEndEffector instructions [here](https://github.com/ADVRHumanoids/ROSEndEffector/tree/devel) to run the gazebo simulation togheter with the main component.
By the way, if you want to test/play/improve this repo, here are the steps:
## Dependencies for Mimic Joints
If mimic joint are present, clone and add to the workspace (and compile together with this package) also the [mimic_joint_plugin](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins). 

#### To run
```bash
roslaunch rosee_gazebo_plugins gazebo.launch hand_name:=<YOUR_HAND_NAME>
```
#### To run also the dynamic reconfigurator:
```bash
rosrun rosee_gazebo_plugins DynReconfigure <YOUR_HAND_NAME>
```
#### Also useful
```bash
rqt
```
And set it to have things like that, for example to tune the gains: 
<p align="center">
<img src="images/rqt.png" width="700">
</p>

### How to run with your model
* Be sure to have an urdf file ready for gazebo [info](http://gazebosim.org/tutorials/?tut=ros_urdf)   
    Also add in your urdf :
    ```xml
    <gazebo>
        <plugin name="SOMENAME" filename="librosee_plugin.so"> </plugin>
    </gazebo>
    ```
* Create new *YOURROBOTNAME_control.yaml* file in *configs* folder, following the examples present there. Note that supported controllers *type* are *JointPositionController* and *JointVelocityController* (but rosee core at the moment send only position references)

* **Important**: If your urdf file has some mimic joint, you have to add a  [mimic_joint_plugin](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins) for each mimic joint. See [two_finger_mimic.urdf.xacro](https://github.com/ADVRHumanoids/rosee_gazebo_plugins/blob/master/configs/urdf/two_finger_mimic.urdf.xacro) file for example on how to use the plugin, or also [mimic_joint_gazebo_tutorial](https://github.com/mintar/mimic_joint_gazebo_tutorial)

* Check above if you want to run the dynamic reconfigurator too

## Change more params with Dynamic Reconfigurator
* Check the ros tutorials about that ( [here](http://wiki.ros.org/dynamic_reconfigure/Tutorials) ) 
* Add (or extend) config files in *cfg* folder
* Check the DynReconfigure code

