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
    Desc: Control Loop for the 6DOF cooking robot arm "ADELE"
*/

#include<adele_control_2/adeleControlLoop.h>

#include<rosparam_shortcuts/rosparam_shortcuts.h>

//This code has largely been repurposed from the ros_control_boilerplate generic_hw_control_loop script
//LINK: https://github.com/PickNikRobotics/ros_control_boilerplate

namespace adele_control_2{

AdeleHWControlLoop::AdeleHWControlLoop(ros::NodeHandle& nh, 
                                        std::shared_ptr<hardware_interface::RobotHW> hardwareInterface)
                                        :nh_(nh), hardware_interface_(hardwareInterface)
{
    // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

  // Load rosparams
  ros::NodeHandle rpsnh(nh, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpsnh, "loop_hz", loopHZ);
  error += !rosparam_shortcuts::get(name_, rpsnh, "cycle_time_error_threshold", cycle_time_error_threshold);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Get current time for use with first update
  clock_gettime(CLOCK_MONOTONIC, &lastTime);

  desired_update_period = ros::Duration(1 / loopHZ);
}

void AdeleHWControlLoop::run(){
    ros::Rate rate(loopHZ);
    while (ros::ok())
    {
        update();
        rate.sleep();
    }
}

void AdeleHWControlLoop::update()
{
  // Get change in time
  clock_gettime(CLOCK_MONOTONIC, &currentTime);
  elapsed_time =
      ros::Duration(currentTime.tv_sec - lastTime.tv_sec + (currentTime.tv_nsec - lastTime.tv_nsec) / BILLION);
  lastTime = currentTime;
  ros::Time now = ros::Time::now();
  // ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "generic_hw_main","Sampled update loop
  // with elapsed time " << elapsed_time_.toSec());

  // Error check cycle time
  const double cycle_time_error = (elapsed_time - desired_update_period).toSec();
  if (cycle_time_error > cycle_time_error_threshold)
  {
    ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
                                     << cycle_time_error << ", cycle time: " << elapsed_time
                                     << ", threshold: " << cycle_time_error_threshold);
  }

  // Input
  hardware_interface_->read(now, elapsed_time);

  // Control
  controller_manager_->update(now, elapsed_time);

  // Output
  hardware_interface_->write(now, elapsed_time);
}

}
