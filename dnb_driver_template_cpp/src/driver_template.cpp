#include <driver_template.h>

bool DriverTemplate::shutdown_requested = false;

DriverTemplate::DriverTemplate() {
    ros::NodeHandle private_nh("~");

    // initialize shutdown signal handler
    signal((int) SIGINT, DriverTemplate::shutdown_signal);

    //initialize ROS Publishers, Subscribers and ServiceServers
    sub_command_list = nh.subscribe("command_list", 1, &DriverTemplate::callback_rmi_command, this);
    pub_command_result = nh.advertise<robot_movement_interface::Result>("command_result", 1);

    srv_stop_robot = nh.advertiseService("stop_robot_right_now", &DriverTemplate::callback_stop_robot, this);

    pub_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    pub_tool_frame = nh.advertise<robot_movement_interface::EulerFrame>("tool_frame", 1);
    pub_robot_status = nh.advertise<industrial_msgs::RobotStatus>("robot_status", 1);
    pub_status = private_nh.advertise<dnb_msgs::ComponentStatus>("status", 5, true);

    DriverTemplate::connect_robot();
    ROS_INFO("driver initialized");
}

DriverTemplate::~DriverTemplate(){
    ROS_INFO("driver shutdown");
    DriverTemplate::disconnect_robot();
    command_list.clear();
    command_list_launched.clear();
}

void DriverTemplate::shutdown_signal(int signal){
    shutdown_requested = true;
}

void DriverTemplate::spin(){
    ros::Rate loop_rate(100);

    dnb_msgs::ComponentStatus cm_status;
    cm_status.status_id = dnb_msgs::ComponentStatus::RUNNING;
    cm_status.status_msg = "Running";
    pub_status.publish(cm_status);

    while(ros::ok() && !shutdown_requested){
        // launch new commands
        if (command_list.size() > 0){
            // process the new movement command
            IndividualCommandTemplate indiv_command = DriverTemplate::process_command(command_list[0]);

            // start the new movement command
            DriverTemplate::start_motion(indiv_command);

            // move the command from the open list to the launched list
            command_list_launched.push_back(command_list[0]);
            command_list.pop_front();
        }

        if (command_list_launched.size() > 0){

            robot_movement_interface::Result result_msg;
            result_msg.command_id = command_list_launched[command_list_launched.size() - 1].command_id;

            // check whether the current waypoint was reached
            // <!> reachedWaypointByChangingTarget: disable this function if your controller does not provide the current target waypoint pose (see get_current_waypoint() function below for further information)
            // <disable_begin>
            if (!equals(array_to_tf(command_list_launched[0].pose), euler_to_tf(DriverTemplate::get_current_waypoint()))){
                command_list_launched.pop_front();
            }
            // <disable_end>

            // check, whether the robot is stopped. This indicates, whether the current trajectory has been completed.
            if (DriverTemplate::is_stopped()){
                // Check if the robot was correctly finished. If the commands finished correctly, then result_code is 0
                // if not finishes correctly, result_code must be -1
                if (DriverTemplate::in_error()){
                    result_msg.result_code = -1;
                } else {
                    result_msg.result_code = 0;
                }
                pub_command_result.publish(result_msg);

                command_list_launched.clear();
            }
        }

        DriverTemplate::publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    cm_status.status_id = dnb_msgs::ComponentStatus::STOPPED;
    cm_status.status_msg = "Stopped";
    pub_status.publish(cm_status);
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
    robot_movement_interface::EulerFrame tool_frame = DriverTemplate::get_current_robot_pose();
    pub_tool_frame.publish(tool_frame);

    // tf
    try {
        broadcaster.sendTransform(tf::StampedTransform(euler_to_tf(tool_frame), ros::Time::now(), "manufacturer_base", "robot_state_tcp"));
    } catch (tf::TransformException &e) {
        ROS_WARN("Cannot send tf broadcast: %s", e.what());
    }

    // Robot status
    industrial_msgs::RobotStatus status;
    status.header.stamp = ros::Time::now();

    // <!> Fill here the current robot mode (here in pseudo code)
    // Example:
    status.mode.val = 2; // UNKNOWN = -1, MANUAL = 1, AUTO = 2; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/RobotMode.msg
    status.e_stopped.val = 0; // UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
    status.drives_powered.val = 1; // UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
    status.motion_possible.val = 1; // UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
    status.in_motion.val = 0; // UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
    status.in_error.val = 0; // UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState

    pub_robot_status.publish(status);
}

bool DriverTemplate::callback_stop_robot(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    ROS_INFO("stop robot request received");
    // cancel current command list
    command_list.clear();
    command_list_launched.clear();

    // stop robot and delete all pending movement commands; it is absolutely necessary to speed the robot down to zero
    DriverTemplate::halt_motion();

    res.success = true;
    return true;
}

void DriverTemplate::callback_rmi_command(const robot_movement_interface::CommandListConstPtr &msg) {
    ROS_INFO("rmi request received");

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

void DriverTemplate::connect_robot(){
    // <!> Put here your code to connect the robot to the driver
    ROS_INFO("driver connected");
}

void DriverTemplate::disconnect_robot(){
    // <!> Put here your code to disconnect the robot from the driver
    ROS_INFO("driver disconnected");
}

IndividualCommandTemplate DriverTemplate::process_command(robot_movement_interface::Command &command) {
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

robot_movement_interface::EulerFrame DriverTemplate::get_current_robot_pose(){
    // <!> Put here the current robot pose (flange pose in robot base coordiantes) sent by the controller
    // Example:
    robot_movement_interface::EulerFrame current_pose;
    current_pose.x = 0.5;
    current_pose.y = 0.0;
    current_pose.z = 0.5;
    current_pose.alpha = 0.0;
    current_pose.beta = 0.0;
    current_pose.gamma = 0.0;
    return current_pose;
}

robot_movement_interface::EulerFrame DriverTemplate::get_current_waypoint(){
    // <!> Put here the current waypoint pose the robot currently drives to (flange pose in robot base coordiantes) sent by the controller.
    // All waypoints of a trajectory are sent to the robot controller with start_motion commands and then processed one after another.
    // When the robot processes the trajectory, it steps through all waypoints. The robot discards the current waypoint and continues with the next one, when he reached the first waypoint's position.
    // If your controller does not provide the current waypoint, the driver can't pause the motion and resume it later with the last reached waypoint.
    // In this case leave the function empty or delete it and comment or remove the 'reachedWaypointByChangingTarget' in the 'spin()' function.
}

tf::Pose DriverTemplate::array_to_tf(std::vector<float> &array){
    tf::Pose pose;
    pose.setOrigin(tf::Vector3(array[0], array[1], array[2]));
    pose.setRotation(tf::createQuaternionFromRPY(array[5], array[4], array[3]));
    return pose;
}

tf::Pose DriverTemplate::euler_to_tf(robot_movement_interface::EulerFrame euler){
    tf::Pose pose;
    pose.setOrigin(tf::Vector3(euler.x, euler.y, euler.z));
    pose.setRotation(tf::createQuaternionFromRPY(euler.gamma, euler.beta, euler.alpha));
    return pose;
}

bool DriverTemplate::equals(tf::Pose p1, tf::Pose p2, double limit_tlat, double limit_rot){
    tf::Vector3 diff_vec_tlat = p1.getOrigin() - p2.getOrigin();
    double delta_tlat = diff_vec_tlat.length();
    double delta_rot = tf::angleShortestPath(p1.getRotation(), p2.getRotation());
    return ((delta_tlat < limit_tlat) && (delta_rot < limit_rot));
}

// ##################################################################################################################################################

int main(int argc, char **argv){
    ros::init(argc, argv, "dnb_driver_template_cpp");

    DriverTemplate driver;
    driver.spin();

    return 0;
}
