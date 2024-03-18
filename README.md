# Adele-ROS
The ROS packages for running the Adele 6DOF robotic arm.

# Prerequisite packages

1. [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. [Moveit!](https://moveit.ros.org/install/)
3. [ros_control](http://wiki.ros.org/ros_control#Install)
4. [ros-control-boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate)

To check whether you have the prerequisites installed, first run:
> source path_to_your_workspace/devel/setup.bash
>
> roscore

Then in a separate window, run:
> rospack list

If you can find all of the following in the list, you're good to go:
- moveit
- ros_control
- ros_control_boilerplate

# Installing prerequisites
The Adele-ROS project requires Boost Library installed to access the shared_ptr library. All other prerequisites are installed with the ROS install.

# Documentation
The Adele-ROS workspace has 3 main packages:
1. adele_control_2, the RobotHW package (for ros_control use)
2. adele_moveit_config, the moveit package (for moveit to access the URDF)
3. adele_urdf_2, the URDF package (provides the robot model and data)

# adele_control_2

There are three critical scripts for adele_control_2:
1. [adeleHWInterface.cpp](#AdeleHWInterface)
2. [adeleMain.cpp](#FollowJointTrajectoryServer)
3. AdeleControlLoop.cpp

# AdeleHWInterface
Serves to define the AdeleHW class, which is derived from the GenericHWInterface Class.
## AdeleHW(const ros::NodeHandle& nh, urdf::Model* urdf_model)
---
Constructor, accepts 2 parameters.
- & nh: accepts the calling script's node handle by reference
- \* urdf_model: a pointer to the urdf model object 
  - can also be provided as NULL, in which case, the script will try to read the urdf from the ROS parameter server

The AdeleHW() constructor has a few responsibilities:
1. It needs to find the URDF from the rosparam server if it is not specified
2. It needs to generate lists of all the actuator and joint names for the object
3. It needs to set up any publishers/subscribers the node will need for running the robot

The loading of the URDF and the generation of joint names are taken care of by the GenericHWInterface class constructor. However, the AdeleHW constructor also needs to load the URDF in string form in order to set up the transmission objects

### 1. Loading the URDF string
---
```cpp
  loadURDFString(nh, "robot_description");
```
The constructor uses the provided node handle to retrieve the parameter "/robot_description" (the URDF) from the ROS parameter server. You will need to use a launch file to upload the URDF as a parameter with this exact name to the parameter server.

### 2. Generating lists of all actuator names
---
```cpp
controllerManager.reset(new controller_manager::ControllerManager(this, nh_));
    
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "actuators", actuator_names_);
    rosparam_shortcuts::shutdownIfError(name_, error);
    ROS_INFO_STREAM("Actuator and joint params retrieved.");
```
Using a node handle derived from the supplied node handle, the constructor retrieves params **"actuators"**  of namespace **name_** (in this case, **"adele_hw_interface"**) from the ros parameter server. If the list cannot be retrieved, the node will shut itself down and throw an error.

### 3. Setting up publishers and subscribers
---
One publisher and one subscriber each are required for serial communication with the Teensy microcontroller.
1. **telemetrySub** listens for position feedback from the Teensy
2. **trajPublisher** publishes position commands to the Teensy
Both of these use custom ROS messages, which we will expand on later.

# initializeHardware()
Starts the object's controllers, calls the methods **registerActuatorInterfaces()** and **loadTransmissions()**. 

Outputs:
- true on success
- false on failure of **loadTransmissions()**

# registerActuatorInterfaces()
Generates structs (another C object type) which contain all the data/storage variables necessary for each actuator controller.

You can find the definition of the struct **JointWithPos** in the header file [adeleHWInterface.h](src/adele_control_2/include/adele_control_2/adeleHWInterface.h). The properties of the struct are listed below.
```cpp
  double position;
  double velocity;
  double effort;
  double command;
  hardware_interface::ActuatorStateHandle stateHandle;
  hardware_interface::ActuatorHandle handle;
  double jointPos;
```
The JointWithPos structs are stored in a length 6 array in AdeleHW, called **actuators**.

In order to register the interfaces, the handles for each actuator need to be registered first. The ActuatorStateHandle of the actuator keeps track of the actuator's current state (as the name suggests) and is linked to four variables:
1. The actuator's name (in a string array stored in AdeleHW)
2. position (part of the struct)
3. velocity (part of the struct)
4. effort (part of the struct)

It is registered as such:
```cpp
  hardware_interface::ActuatorStateHandle act_state_handle(actuator_names_[i], &actuators[i].position, &actuators[i].velocity, &actuators[i].effort);

  actuators[i].stateHandle = act_state_handle;
``` 

The ActuatorHandle keeps track of the commands (in this case, position commands) issued to the actuator and the actuator's state. As such, it is linked to two variables:
1. The actuator's ActuatorStateHandle (part of the struct)
2. command (part of the struct)

```cpp
  actuators[i].handle = hardware_interface::ActuatorHandle(actuators[i].stateHandle, &actuators[i].command);
  act_state_interface_.registerHandle( actuators[i].stateHandle );
  pos_act_interface_.registerHandle(actuators[i].handle);
```
The last two lines serve to register each handle under its respective interface:
1. **act_state_interface_**: the ActuatorToJointStateInterface
2. **pos_act_interface_**: the JointToActuatorPositionInterface

These interfaces do exactly what it says on the tin: They facilitate translating actuator position states to joint position states, and joint position commands to actuator position commands.


So if you need to access the current position of the i-th actuator, you would call it by:
```cpp
actuators[i].position
```
Writing a command to an actuator is a little more complicated because of the necessity to run a translation from joint to actuator position.
```cpp
handle = this->get<hardware_interface::PositionJointInterface>()->getHandle(joint_names_[linkNo])
```
This method to get a handle for the controller of the joint you need is slightly hacky, but it does work. Once you can get ahold of said handle, you can easily upload a position command with:
```
handle.setCommand(value)
```
This command value will be translated to a actuator command value by the write() method, which we will discuss later.

Note that the hardware interfaces here will serve as your required **action server** for moveit to make requests from.

