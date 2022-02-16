/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/* Author: Timothy Ng
    Desc: Hardware Interface for the 6DOF cooking robot arm "ADELE"
*/

#include<adele_control_2/adeleHWInterface.h>
#include<ros/console.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>


//This code has largely been repurposed from the ros_control_boilerplate generic_hw_interface script
//LINK: https://github.com/PickNikRobotics/ros_control_boilerplate

namespace adele_control_2
{


AdeleHW::AdeleHW(const ros::NodeHandle& nh, urdf::Model* urdf_model):
    GenericHWInterface(nh, urdf_model), name_("adele_hw_interface")
{
    // if (urdf_model == NULL)
    //     loadURDF(nh, "/robot_description");
    // else
    //     urdf_model_ = urdf_model;

  // Load rosparams
    loadURDFString(nh, "robot_description");
    ros::NodeHandle rpnh(
      nh_, "hardware_interface");
    ROS_INFO_STREAM("Param access nodeHandle generated");
    
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "actuators", actuator_names_);
    // error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
    rosparam_shortcuts::shutdownIfError(name_, error);
    ROS_INFO_STREAM("Actuator and joint params retrieved.");
    
    debug = true;
    //directControl = true;
    telemetrySub = nh_.subscribe<adele_control_2::adeleTelemetry>("/teensy/adeleTelem", 1, &AdeleHW::callBackFn, this);
    trajPublisher = nh_.advertise<adele_control_2::armComd>("/teensy/armCmd", 1);
    //ros::ServiceClient jntTraj = nh_.serviceClient<control_msgs::FollowJointTrajectoryAction>("followTraj");
    ROS_INFO_STREAM("Constructor Success");
}

bool AdeleHW::loadTransmissions(){
        using namespace transmission_interface;
        ROS_INFO_STREAM("Transmission interface loading begin.");
  // Initialize transmission loader
        try
        {
            transmission_loader_.reset(new TransmissionInterfaceLoader(this, &transmissions_));
        }
        catch(const std::invalid_argument& ex)
        {
            ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
            return false;
        }
        catch(const pluginlib::LibraryLoadException& ex)
        {
            ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
            return false;
        }
        catch(...)
        {
            ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
            return false;
        } 

        ROS_INFO_STREAM("Interface loader created.");
  // Perform actual transmission loading
        if (!transmission_loader_->load(urdf_string)) {return false;}
        ROS_INFO_STREAM("Loaded transmissions from URDF");

  // Get the transmission interfaces
        act_to_jnt_state_ = transmissions_.get<ActuatorToJointStateInterface>();
        p_jnt_to_act_pos_ = transmissions_.get<JointToActuatorPositionInterface>();
        ROS_INFO_STREAM("Transmission interfaces latched.");
        return true;
}


void AdeleHW::registerActuatorInterfaces(){
    // ROS_INFO_STREAM("Attempting to register " << numActuators << " actuators:");
    // for (std::size_t i = 0; i < numActuators; i++){
    //     ROS_INFO_STREAM(actuator_names_[i] << " " << i);
    // }
    
    for (std::size_t i = 0; i < num_joints_; i++){
        ROS_INFO_STREAM("Starting registration of actuator "<< i );
        hardware_interface::ActuatorStateHandle act_state_handle(actuator_names_[i], 
            &actuators[i].position, &actuators[i].velocity, &actuators[i].effort);
        actuators[i].stateHandle = act_state_handle;
        ROS_INFO_STREAM("State handle registered for actuator "<< i );
        actuators[i].handle = hardware_interface::ActuatorHandle(actuators[i].stateHandle, &actuators[i].command);
        act_state_interface_.registerHandle( actuators[i].stateHandle );
        pos_act_interface_.registerHandle(actuators[i].handle);
        ROS_INFO_STREAM("Actuator "<< i <<"'s handles registered with interface.");
    }

    ROS_INFO_STREAM("All handles registered");

    try{
        registerInterface(&act_state_interface_);
    }
    catch(std::logic_error e){
        ROS_ERROR_STREAM("Interface registration failed, cause: " << e.what());
        ros::shutdown();
    }
    catch(...){
        ROS_ERROR_STREAM("Not quite sure what went wrong, but interface registration failed. Stand far away.");
        ros::shutdown();
    }
    

    ROS_INFO_STREAM("Actuator state interfaces registered.");
    registerInterface(&pos_act_interface_);
    ROS_INFO_STREAM("All interfaces registered.");
}

bool AdeleHW::initializeHardware(){
// Register interfaces with the RobotHW interface manager, allowing ros_control operation
    ROS_INFO_STREAM("Beginning Hardware Initialization");
    
    num_joints_ = joint_names_.size();
    registerActuatorInterfaces();
    // Load transmission information from URDF
    if(!loadTransmissions()){return false;}
    //init();
    // joint_state_interface_ = *this->get<hardware_interface::JointStateInterface>();
    // position_joint_interface_ = *this->get<hardware_interface::PositionJointInterface>();
    
    setHardwareInterfaces();

    if(debug){
        ROS_INFO_STREAM("Debugging enabled");
        hardware_interface::JointHandle backdoorHandle; 
        try{
            backdoorHandle = this->get<hardware_interface::PositionJointInterface>()->getHandle(joint_names_[1]);
        }
        catch(...){
            ROS_ERROR_STREAM("HANDLE NOT LATCHED");
        }
        ROS_INFO_STREAM("Latched handle");
    
        backdoorHandle.setCommand(0.785);
        ROS_INFO_STREAM("Joint "<<joint_names_[1]<<" command: "<<backdoorHandle.getCommand());
        p_jnt_to_act_pos_->propagate();
        ROS_INFO_STREAM("Command for "<<actuators[1].command<<" to be sent to actuator.");
        ROS_INFO_STREAM("Debug end.");
        backdoorHandle.setCommand(0.0);
    }

    return true;
}

bool AdeleHW::setHardwareInterfaces(){
    hardware_interface::JointHandle handles[num_joints_];

    for(int i = 0; i < num_joints_; i++){
        handles[i] = this->get<hardware_interface::PositionJointInterface>()->getHandle(joint_names_[i]);
        handles[i].setCommand(0.0);
    }
    writeCommandsToHardware();
    updateJointsFromHardware();
    
    return true;

}

void AdeleHW::loadURDFString(const ros::NodeHandle& nh, std::string param_name)
{
    urdf_model_ = new urdf::Model();
    
    // search and wait for robot_description on param server
    while (urdf_string.empty() && ros::ok())
    {
    
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name))
    {
        ROS_INFO_STREAM_NAMED(name_, "Parameter found. Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                                    << search_param_name);
        
        try{
            nh.getParam(search_param_name, urdf_string);
        }
        catch(std::logic_error e){
            ROS_ERROR_STREAM(e.what());
            ROS_INFO_STREAM("Extracted parameter: " << urdf_string);
        }
        catch(...){
            ROS_ERROR_STREAM("Parameter Extraction failed.");
        }
        ROS_INFO_STREAM("xml string retrieved.");
    }
    else
    {
        ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                                    << param_name);
        
        try{
            nh.getParam(param_name, urdf_string);
        }
        catch(std::logic_error e){
            ROS_ERROR_STREAM(e.what());
            ROS_INFO_STREAM("Extracted parameter: " << urdf_string);
        }
        catch(...){
            ROS_ERROR_STREAM("Parameter Extraction failed.");
        }
        
    }
    

    usleep(100000);
    
} //while
} //loadURDFString

/*
void AdeleHW::reset()
{
  // Reset joint limits state, in case of mode switch or e-stop
}
*/

void AdeleHW::callBackFn(const adele_control_2::adeleTelemetry::ConstPtr& telemetry){
    /* msg structure for reference

    float32[6] efforts  # amps
    float32[6] vels     # rad/s
    float32[6] pos      # rad
    time startSyncTime 
    uint32 isrTicks # this would overflow if the robot is left on for 497 days straight at 100 hz 
    uint8 bufferHealth
    
    */

   /*
    Reminder for the actuator states:
    struct JointWithPos{
        double position;
        double velocity;
        double effort;
        double command;
        hardware_interface::ActuatorStateHandle stateHandle;
        hardware_interface::ActuatorHandle handle;
        double jointPos;}
   */
    for(size_t i = 0; i < num_joints_; i++){
        actuators[i].position = telemetry->pos[i];
        actuators[i].velocity = telemetry->vels[i];
    }
    updateJointsFromHardware();
}

void AdeleHW::updateJointsFromHardware(){
    act_to_jnt_state_->propagate();

}

void AdeleHW::writeCommandsToHardware(){
    p_jnt_to_act_pos_->propagate();
    
    //TODO: provide script to write command
    
}

void AdeleHW::read(ros::Duration& elapsed_time){
    ros::spinOnce();
    //trajPublisher.publish()
}

void AdeleHW::write(ros::Duration& elapsed_time){
    writeCommandsToHardware();
    static adele_control_2::armComd armCmd;
    /*
    float32[6] efforts  # amps
    float32[6] vels     # rad/s
    float32[6] pos      # rad
    uint32 msgCounter   # counter to check for missed messages
    */
    for(size_t i = 0; i < num_joints_; i++){
        armCmd.pos[i] = actuators[i].command;
    }
    
    trajPublisher.publish(armCmd);

    //trajPublisher.publish(trajGoal);
}

void AdeleHW::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
}

// void AdeleHW::directCommandWrite(int linkNo, double commandValue){
//     ROS_INFO_STREAM("Attempting to directly write value "<< commandValue <<" to joint " << joint_names_[linkNo]);
//     if(directControl){
//         hardware_interface::JointHandle backdoorHandle; 
//         try{
//             backdoorHandle = this->get<hardware_interface::PositionJointInterface>()->getHandle(joint_names_[linkNo]);
//         }
//         catch(...){
//         ROS_ERROR_STREAM("HANDLE NOT LATCHED");
//         }
//         ROS_INFO_STREAM("Latched handle");
        
//         backdoorHandle.setCommand(commandValue);
//         ROS_INFO_STREAM("Joint "<<joint_names_[linkNo]<<" command: "<<backdoorHandle.getCommand());
//     }    
//     else{
//         ROS_ERROR_STREAM("Direct Control has NOT been enabled!");
//     }
// }

// double AdeleHW::directCommandAccess(int linkNo){
//     if(directControl){
//         ROS_INFO_STREAM(actuators[linkNo].command); 
//         return actuators[linkNo].command;
//     }
//     else{
//         ROS_ERROR_STREAM("Direct Control has NOT been enabled!");
//         return 0;
//     }
// }

bool AdeleHW::checkForConflict(...) const {
    return false;
}

}
/*
void AdeleHW::update(const ros::TimerEvent& ev){
    elapsed_time = ros::Duration(ev.current_real-ev.last_real);
    read(elapsed_time);
    controllerManager->update(ros::Time::now(), elapsed_time);
    write(elapsed_time);
}
*/
