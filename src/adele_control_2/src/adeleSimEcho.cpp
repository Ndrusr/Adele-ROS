#include <ros/ros.h>
#include <adele_control_2/armComd.h>
#include <adele_control_2/adeleTelemetry.h>

ros::Publisher callbackPub;

void cmdCallback(const adele_control_2::armComd::ConstPtr &msg){
    /* 
    ARM COMD
        float32[6] efforts  # amps
        float32[6] vels     # rad/s
        float32[6] pos      # rad
        uint32 msgCounter   # counter to check for missed messages
    
    TELEM
        float32[6] efforts  # amps
        float32[6] vels     # rad/s
        float32[6] pos      # rad
        time startSyncTime 
        uint32 isrTicks # this would overflow if the robot is left on for 497 days straight at 100 hz 
        uint8 bufferHealth
    */
   static adele_control_2::adeleTelemetry telemetry;

   for(int i = 0; i<msg->pos.size();i++){
       telemetry.pos[i] = msg->pos[i];
       telemetry.vels[i] = msg->vels[i];
       telemetry.efforts[i] = msg->efforts[i];
   }

    callbackPub.publish(telemetry);


}

int main(int argc, char** argv){
    ros::init(argc, argv, "adele_sim_echo");

    ros::NodeHandle nh;

    ros::Subscriber comd_sub = nh.subscribe<adele_control_2::armComd>("/teensy/armCmd", 10, cmdCallback);

    callbackPub = nh.advertise<adele_control_2::adeleTelemetry>("/teensy/adeleTelem", 10);

    ros::spin();

    return 0;

}