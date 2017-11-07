// ----------------------------------------------------------------------------
// Copyright 2017 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
// This file contains the driver logic
// ----------------------------------------------------------------------------

#ifndef DRIVER_TEMPLATE_H_
#define DRIVER_TEMPLATE_H_

#include <thread>
#include <mutex>
#include <atomic>

#include <string.h>

#include <tf/tf.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <industrial_msgs/RobotStatus.h>

#include <robot_movement_interface/Result.h>
#include <robot_movement_interface/Command.h>
#include <robot_movement_interface/EulerFrame.h>
#include <robot_movement_interface/CommandList.h>
#include <robot_movement_interface/GetIO.h>
#include <robot_movement_interface/SetIO.h>
#include <robot_movement_interface/SetFloat.h>

#include <individual_command_template.h>

#include <rmi_lib.h>


class DriverTemplate {
    public:
        DriverTemplate();
        ~DriverTemplate();
        void spin();

    private:

        /*
         * Class Variables
         */
        ros::NodeHandle nodeHandle;

        float speed_factor;
        std::deque<robot_movement_interface::Command> command_list;
        std::deque<robot_movement_interface::Command> command_list_launched;
        std::mutex command_list_mutex;

        std::atomic<bool> restart_requested;

        /*
         * ROS Publishers, Subscribers and ServiceServers
         */
        ros::Subscriber sub_command_list;
        ros::Publisher pub_command_result;

        ros::ServiceServer srv_get_io;
        ros::ServiceServer srv_set_io;
        ros::ServiceServer srv_scale_speed;
        ros::ServiceServer srv_stop_robot;

        ros::Publisher pub_joint_states;
        ros::Publisher pub_tool_frame;
        ros::Publisher pub_robot_status;

        /*
         * ROS callback functions
         */
        void callback_rmi_command(const robot_movement_interface::CommandListConstPtr &msg);
        bool callback_stop_robot(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool callback_set_io(robot_movement_interface::SetIO::Request &req, robot_movement_interface::SetIO::Response &res);
        bool callback_get_io(robot_movement_interface::GetIO::Request &req, robot_movement_interface::GetIO::Response &res);
        bool callback_scale_speed(robot_movement_interface::SetFloat::Request &req, robot_movement_interface::SetFloat::Response &res);

        /*
         * Basic driver functions
         */
        void connectRobot();
        void disconnectRobot();
        IndividualCommandTemplate processCommand(robot_movement_interface::Command &command);
        void start_motion(IndividualCommandTemplate &indiv_command);
        void stop_motion();
        bool is_stopped();
        tf::Pose get_current_robot_pose();
        tf::Pose get_current_waypoint();

        /*
         * Other functions
         */
        static void cb_thread();
        void publish();


};

#endif
