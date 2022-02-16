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

#ifndef ADELE_CONTROL_2_ADELEHWINTERFACE
#define ADELE_CONTROL_2_ADELEHWINTERFACE

#include <ros_control_boilerplate/generic_hw_interface.h>

#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <adele_control_2/adeleTelemetry.h>
#include <adele_control_2/armComd.h>

#include<transmission_interface/transmission_interface_loader.h>
#include<array>
#include<vector>
#include<boost/shared_ptr.hpp>
#include<string>

//This code has largely been repurposed from the ros_control_boilerplate generic_hw_interface script
//LINK: https://github.com/PickNikRobotics/ros_control_boilerplate

namespace adele_control_2{
class AdeleHW: public ros_control_boilerplate::GenericHWInterface{
public:
   
    AdeleHW(const ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

    bool initializeHardware();
    
    void updateJointsFromHardware();
    void writeCommandsToHardware();
    
    
    virtual void read(ros::Duration& elapsed_time);
    virtual void write(ros::Duration& elapsed_time);

    //void reset();

    //void directCommandWrite(int linkNo, double commandValue);
    //double directCommandAccess(int linkNo);

    virtual void enforceLimits(ros::Duration& period);

    virtual bool checkForConflict(...) const;

private:

    hardware_interface::ActuatorStateInterface act_state_interface_;
    hardware_interface::PositionActuatorInterface pos_act_interface_;

    transmission_interface::RobotTransmissions transmissions_;
    std::unique_ptr <transmission_interface::TransmissionInterfaceLoader> transmission_loader_;
    transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state_;
    transmission_interface::JointToActuatorPositionInterface* p_jnt_to_act_pos_;
    
    struct JointWithPos{
        double position;
        double velocity;
        double effort;
        double command;
        hardware_interface::ActuatorStateHandle stateHandle;
        hardware_interface::ActuatorHandle handle;
        double jointPos;

        JointWithPos() : 
         position(0.0), velocity(0.0), effort(0.0), command(0.0)
        {}

        JointWithPos(double pos):
            position(pos), velocity(0), effort(0), command(0)
        {}
    };
    JointWithPos actuators[6];
    
    //std::vector<std::string> joint_names_;
    std::vector<std::string> actuator_names_;




protected:
    std::string name_;

    virtual void loadURDFString(const ros::NodeHandle& nh, std::string param_name);
    //ros::NodeHandle nh_;
    std::string urdf_string;
    //urdf::Model* urdf_model_;

    void registerActuatorInterfaces();

    bool loadTransmissions();

    bool setHardwareInterfaces();

    ros::Subscriber telemetrySub;
    void callBackFn(const adele_control_2::adeleTelemetry::ConstPtr& telemetry);
    ros::Publisher trajPublisher;

    // ros::Timer my_control_loop;
    ros::Duration elapsed_time;
    //double loopHz;
    //boost::shared_ptr<controller_manager::ControllerManager> controllerManager;
    bool debug;
    //bool directControl;
};
}
#endif 