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

#include <driver_template.h>

bool DriverTemplate::shutdown_requested = false;

DriverTemplate::DriverTemplate() {

    // initialize shutdown signal handler
    signal((int) SIGINT, DriverTemplate::shutdownSignal);

    /*
     * initialize ROS Publishers, Subscribers and ServiceServers
     */
    sub_command_list = nodeHandle.subscribe("command_list", 1, &DriverTemplate::callback_rmi_command, this);
    pub_command_result = nodeHandle.advertise<robot_movement_interface::Result>("command_result", 1);

    srv_get_io = nodeHandle.advertiseService("get_io", &DriverTemplate::callback_get_io, this);
    srv_set_io = nodeHandle.advertiseService("set_io", &DriverTemplate::callback_set_io, this);
    srv_scale_speed = nodeHandle.advertiseService("scale_speed", &DriverTemplate::callback_scale_speed, this);
    srv_stop_robot = nodeHandle.advertiseService("stop_robot_right_now", &DriverTemplate::callback_stop_robot, this);

    pub_joint_states = nodeHandle.advertise<sensor_msgs::JointState>("joint_states", 1);
    pub_tool_frame = nodeHandle.advertise<robot_movement_interface::EulerFrame>("tool_frame", 1);
    pub_robot_status = nodeHandle.advertise<industrial_msgs::RobotStatus>("robot_status", 1);

    /*
     * initialize variables
     */
    speed_factor = 1.0;



    DriverTemplate::connectRobot();
    ROS_INFO_NAMED("driver", "driver initialized");
}

DriverTemplate::~DriverTemplate(){
    ROS_INFO_NAMED("driver", "driver shutdown");
    DriverTemplate::disconnectRobot();
    command_list.clear();
    command_list_launched.clear();
}

void DriverTemplate::shutdownSignal(int signal){
    shutdown_requested = true;
}

void DriverTemplate::spin(){
    ros::Rate loop_rate(100);

    while(ros::ok() && !shutdown_requested){
        // launch new commands
        if (command_list.size() > 0){
            // process the new movement command
            IndividualCommandTemplate indiv_command = DriverTemplate::processCommand(command_list[0]);

            // start the new movement command
            DriverTemplate::start_motion(indiv_command);

            // move the command from the open list to the launched list
            command_list_launched.push_back(command_list[0]);
            command_list.pop_front();

        }

        if (command_list_launched.size() > 0){
            // check whether the current waypoint was reached
            // <!> disable this function if your controller does not provide the current target waypoint pose (see get_current_waypoint() function below for further information)
            // <disable_begin>
            if (rmi_lib::reachedWaypointByChangingTarget(rmi_lib::arrayToPoseTF(command_list_launched[0].pose), DriverTemplate::get_current_waypoint())){
                command_list_launched.pop_front();
            }
            // <disable_end>

            // check, whether the robot is stopped. This indicates, whether the current trajectory has been completed.
            if (DriverTemplate::is_stopped()){
                command_list_launched.clear();

                robot_movement_interface::Result result_msg;
                result_msg.command_id = command_list_launched[command_list_launched.size() - 1].command_id;
                // Check if the robot was correctly finished. If the commands finished correctly, then result_code is 0
                // if not finishes correctly, result_code must be -1
                if (DriverTemplate::in_error()){
                    result_msg.result_code = -1;
                } else {
                    result_msg.result_code = 0;
                }
                pub_command_result.publish(result_msg);
            }

            // check for a restart
            // <!> disable this function if your controller does not provide the current target waypoint pose (see get_current_waypoint() function below for further information)
            // <disable_begin>
            if (restart_requested){
                // stop the currently pending movement commands; it is not necessary to speed the robot down to zero
                DriverTemplate::stop_motion();

                // restart all already and not still completed movement commands
                for (size_t i = 0; i < command_list_launched.size(); i++){
                    // process the movement command
                    IndividualCommandTemplate indiv_command = DriverTemplate::processCommand(command_list_launched[i]);

                    // start the movement command
                    DriverTemplate::start_motion(indiv_command);
                    restart_requested = false;
                }
            }
            // <disable_end>
        }

        DriverTemplate::publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void DriverTemplate::publish(){
    // Joint states
    sensor_msgs::JointState jointState;
    jointState.header.stamp = ros::Time::now();

    // <!> Fill here the current values of all joints (here in pseudo code)
    /*
     * for (size_t i = 0; i < JOINTS; i++){
     *      jointState.name.push_back("JOINT_NAME_i");
     *      jointState.position.push_back(JOINT_POSE_i);
     *      jointState.velocity.push_back(JOINT_VELOCITY_i);
     *      jointState.effort.push_back(JOINT_EFFORT_i);
     * }
     */

    if (jointState.position.size() > 0 && jointState.velocity.size() > 0){
        pub_joint_states.publish(jointState);
    }

    // Tool frame
    robot_movement_interface::EulerFrame tool_frame = rmi_lib::poseTFToEuler(DriverTemplate::get_current_robot_pose());
    pub_tool_frame.publish(tool_frame);

    // Robot status
    industrial_msgs::RobotStatus status;
    status.header.stamp = ros::Time::now();

    // <!> Fill here the current robot mode (here in pseudo code)
    /*
     * status.mode.val = MODE; // UNKNOWN = -1, MANUAL = 1, AUTO = 2; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/RobotMode.msg
     * status.e_stopped.val = E_STOPPED; // UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
     * status.drives_powered.val = DRIVED_POWERED; // UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
     * status.motion_possible.val = MOTION_POSSIBLE; // UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
     * status.in_motion.val = IN_MOTION; // UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
     * status.in_error.val = IN_ERROR; // UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
     */

    pub_robot_status.publish(status);
}

bool DriverTemplate::callback_stop_robot(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    ROS_INFO_NAMED("driver", "stop robot request received");
    // cancel current command list
    command_list.clear();
    command_list_launched.clear();

    // stop robot and delete all pending movement commands; it is absolutely necessary to speed the robot down to zero
    DriverTemplate::halt_motion();

    res.success = true;
    return true;
}

void DriverTemplate::callback_rmi_command(const robot_movement_interface::CommandListConstPtr &msg) {
    ROS_INFO_NAMED("driver", "rmi request received");

    // if we replace the commands discard all movement commands and stop the robot first
    if (msg->replace_previous_commands){
        command_list.clear();
        command_list_launched.clear();

        // stop robot and delete all pending movement commands; it is not necessary to speed the robot down to zero
        DriverTemplate::stop_motion();
    }

    // build new command list by appending it to the current commands
    for (int i = 0; i < msg->commands.size(); i++) {
        command_list.push_back(msg->commands[i]);
    }
}

// This service sets an IO from ROS. This will stop the robot because replace the current command list
bool DriverTemplate::callback_set_io(robot_movement_interface::SetIO::Request &req, robot_movement_interface::SetIO::Response &res){
    ROS_INFO_NAMED("driver", "Set IO request received");

    // <!> Set here your io's from the service message (here in pseudo code)
    /*
     * IO[req.number] = req.value;
     */

    return true;
}

// This service sets an IO from ROS. This will stop the robot because replace the current command list
bool DriverTemplate::callback_get_io(robot_movement_interface::GetIO::Request &req, robot_movement_interface::GetIO::Response &res){
    ROS_DEBUG_NAMED("driver", "Get IO request received");

    // <!> Get here your io's and write it to the service message (here in pseudo code)
    /*
     * res.value = IO[req.number];
     */

    return true;
}

// This service sets the scale speed variable.
bool DriverTemplate::callback_scale_speed(robot_movement_interface::SetFloat::Request &req, robot_movement_interface::SetFloat::Response &res){
    ROS_DEBUG_NAMED("driver", "Scale speed=%1.2f%% request received", req.value);

    if ((req.value < 0.0) || (req.value > 1.0)){
        ROS_WARN_NAMED("driver", "Scale speed=%1.2f%% is out of bounds and will be cropped to [1%%, 100%%]!", 100 * req.value);
        speed_factor = fmin(fmax(0.01, req.value), 1.0);
    } else if (req.value == 0.0){
        ROS_WARN_NAMED("driver", "Scale speed=0.0%% is out of bounds. Robot stops and cancels movement queue!");

        // stop the currently pending movement commands; it is not necessary to speed the robot down to zero
        DriverTemplate::stop_motion();

        return true;
    } else {
        speed_factor = req.value;
    }

    ROS_INFO_NAMED("driver", "Set speed to %f%%", 100 * speed_factor);

    restart_requested = true;

    return true;
}

void DriverTemplate::connectRobot(){
    // <!> Put here your code to connect the robot to the driver
    ROS_INFO_NAMED("driver", "driver connected");
}

void DriverTemplate::disconnectRobot(){
    // <!> Put here your code to disconnect the robot from the driver
    ROS_INFO_NAMED("driver", "driver disconnected");
}

IndividualCommandTemplate DriverTemplate::processCommand(robot_movement_interface::Command &command) {
    IndividualCommandTemplate indiv_command;
    // <!> Adjust here the general movement command to your individual movement controller command
    return indiv_command;
}

void DriverTemplate::start_motion(IndividualCommandTemplate &indiv_command){
    // <!> Put here your code to execute a not-already running individual movement command.
    // This function should just send the command to the robot controller, where it is queued with the others and processed one after another
    // The function is expected to be non-blocking during the movement.
}

void DriverTemplate::stop_motion(){
    // <!> Put here your code to stop all pending movement commands in the robot controller.
    // The robot doesn't have to be at standstill, when leaving this function.
}

bool DriverTemplate::is_stopped(){
    // <!> Put here your code to check, whether the robot is stoppped or not.
    // The controller is then supposed to have no more pending movement commands.
    // The robot is then supposed to have it's velocities near to zero.
}

void DriverTemplate::halt_motion(){
    // <!> Modify this function to stop the robot to standstill if necessary
    // All pending movement commands should be discarded in the robot controller.
    // The robot should be at standstill, when leaving this function.
    while(!DriverTemplate::is_stopped()){
        DriverTemplate::stop_motion();
    }
}

bool DriverTemplate::in_error(){
    // <!> This function needs to return if robot is in error state or not
    // This can be implemented through a global state variable which is updated after each status message
    return false;
}

tf::Pose DriverTemplate::get_current_robot_pose(){
    // <!> Put here the current robot pose (tool_pose in world coordiantes) sent by the controller
}

tf::Pose DriverTemplate::get_current_waypoint(){
    // <!> Put here the current waypoint pose the robot currently drives to (tool_pose in world coordiantes) sent by the controller.
    // All waypoints of a trajectory are sent to the robot controller with start_motion commands and then processed one after another.
    // When the robot processes the trajectory, it steps through all waypoints. The robot discards the current waypoint and continues with the next one, when he reached the first waypoint's position.
    // If your controller does not provide the current waypoint, the driver can't pause the motion and resume it later with the last reached waypoint.
    // In this case leave the function empty or delete it and comment or remove the 'reachedWaypointByChangingTarget' and 'restart_requested' section in the 'spin()' function.
}

// ##################################################################################################################################################

int main(int argc, char **argv){
    ros::init(argc, argv, "dnb_driver_template_cpp");

    DriverTemplate driver;
    driver.spin();

    return 0;
}
