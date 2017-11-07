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

DriverTemplate::DriverTemplate() {

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
    DriverTemplate::disconnectRobot();
}

void DriverTemplate::spin(){
    ros::Rate loop_rate(100);

    std::thread cbthread (DriverTemplate::cb_thread);

    while(ros::ok()){
        // launch new commands
        command_list_mutex.lock();
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
            // <!> disable this function if your controller does not provide the current target waypoint pose
            if (rmi_lib::reachedWaypointByChangingTarget(rmi_lib::arrayToPoseTF(command_list_launched[0].pose), DriverTemplate::get_current_waypoint())){
                command_list_launched.pop_front();
            }

            if (DriverTemplate::is_stopped()){
                command_list_launched.clear();

                robot_movement_interface::Result result_msg;
                result_msg.command_id = command_list_launched[0].command_id;
                result_msg.result_code = 0;
                pub_command_result.publish(result_msg);
            }

            // check for a restart
            // <!> disable this function if your controller does not provide the current target waypoint pose
            if (restart_requested){
                DriverTemplate::stop_motion();
                for (size_t i = 0; i < command_list_launched.size(); i++){
                    // process the movement command
                    IndividualCommandTemplate indiv_command = DriverTemplate::processCommand(command_list_launched[i]);

                    // start the movement command
                    DriverTemplate::start_motion(indiv_command);
                }
            }
        }
        command_list_mutex.unlock();

        DriverTemplate::publish();

        loop_rate.sleep();
    }

    cbthread.join();
    delete this;
}

void DriverTemplate::cb_thread(){
    ros::spin();
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
    robot_movement_interface::EulerFrame tool_frame;

    // <!> Fill here the current values of the tool frame (here in pseudo code)
    /*
     * tool_frame.x = TOOL_POSE_X;
     * tool_frame.y = TOOL_POSE_Y;
     * tool_frame.z = TOOL_POSE_Z;
     * tool_frame.alpha = TOOL_POSE_ALPHA;
     * tool_frame.beta = TOOL_POSE_BETA;
     * tool_frame.gamma = TOOL_POSE_GAMMA;
     */
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

    // stop robot and delete all pending movement commands
    DriverTemplate::stop_motion();

    res.success = true;
    return true;
}

void DriverTemplate::callback_rmi_command(const robot_movement_interface::CommandListConstPtr &msg) {
    ROS_INFO_NAMED("driver", "rmi request received");

    if (!msg->replace_previous_commands){
        // if we don't replace, we need to wait until the command is finished
        while (command_list.size() > 0 || command_list_launched.size() > 0) {
            // wait until stopped
            usleep(100);
        }
    } else {
        command_list.clear();
        command_list_launched.clear();

        // stop robot and delete all pending movement commands
        DriverTemplate::stop_motion();
    }

    // build new command list
    command_list_mutex.lock();

    command_list.clear();
    command_list_launched.clear();
    for (int i = 0; i < msg->commands.size(); i++) {
        command_list.push_back(msg->commands[i]);
    }

    command_list_mutex.unlock();
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
}

void DriverTemplate::disconnectRobot(){
    // <!> Put here your code to disconnect the robot from the driver
}

IndividualCommandTemplate DriverTemplate::processCommand(robot_movement_interface::Command &command) {
    IndividualCommandTemplate indiv_command;
    // <!> Adjust here the general movement command to your individual movement controller command
    return indiv_command;
}

void DriverTemplate::start_motion(IndividualCommandTemplate &indiv_command){
    // <!> Put here your code to execute a not-already running individual movement command.
    // The function is expected to be non-blocking during the movement.
}

void DriverTemplate::stop_motion(){
    // <!> Put here your code to stop the robot to standstill
    // All pending movement commands should be discarded in the robot controller.
}

bool DriverTemplate::is_stopped(){
    // <!> Put here your code to check, whether the robot is stoppped or not.
    // The controller is then supposed to have no more pending movement commands.
}

tf::Pose DriverTemplate::get_current_robot_pose(){
    // <!> Put here the current robot pose (tool_pose in world coordiantes) sent by the controller
}

tf::Pose DriverTemplate::get_current_waypoint(){
    // <!> Put here the current waypoint pose (tool_pose in world coordiantes) sent by the controller
}

// ##################################################################################################################################################

int main(int argc, char **argv){
    ros::init(argc, argv, "dnb_driver_template_cpp");

    DriverTemplate driver;
    driver.spin();

    return 0;
}
