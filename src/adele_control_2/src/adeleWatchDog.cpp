#include <ros/ros.h>
#include <std_msgs/Bool.h> 
 
ros::Publisher watchdogPub;

std_msgs::Bool watchDog;

void kickDog(std_msgs::Bool::ConstPtr &msg){
    
}