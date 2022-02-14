#ifndef ADELE_CONTROL_2_ADELEHWINTERFACE
#define ADELE_CONTROL_2_ADELEHWINTERFACE

#include <ros_control_boilerplate/generic_hw_interface.h>

#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>


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
   
    AdeleHW(const ros::NodeHandle& nh, urdf::Model* urdf_model);

    bool initializeHardware();
    
    void updateJointsFromHardware(trajectory_msgs::JointTrajectoryPoint &point);
    void writeCommandsToHardware();
    
    
    virtual void read(ros::Duration& elapsed_time);
    virtual void write(ros::Duration& elapsed_time);

    //void reset();

    //void directCommandWrite(int linkNo, double commandValue);
    //double directCommandAccess(int linkNo);

    virtual void enforceLimits(ros::Duration& period);

    virtual bool checkForConflict(...) const;

private:
    void registerActuatorInterfaces();

    bool loadTransmissions();

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
         position(0), velocity(0), effort(0), command(0)
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

    virtual void loadURDF(const ros::NodeHandle& nh, std::string param_name) override;
    //ros::NodeHandle nh_;
    std::string urdf_string;
    //urdf::Model* urdf_model_;


    ros::Publisher trajPublisher;

    // ros::Timer my_control_loop;
    ros::Duration elapsed_time;
    //double loopHz;
    boost::shared_ptr<controller_manager::ControllerManager> controllerManager;
    
    //bool directControl;
};
}
#endif 