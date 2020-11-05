// ----------------------------------------------------------------------------
// This file contains the driver logic
// ----------------------------------------------------------------------------

#ifndef DRIVER_TEMPLATE_H_
#define DRIVER_TEMPLATE_H_

#include <string.h>
#include <signal.h>
#include <deque>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <industrial_msgs/RobotStatus.h>

#include <robot_movement_interface/Result.h>
#include <robot_movement_interface/Command.h>
#include <robot_movement_interface/EulerFrame.h>
#include <robot_movement_interface/CommandList.h>
#include <dnb_msgs/ComponentStatus.h>

#include <individual_command_template.h>


class DriverTemplate {
    public:
        DriverTemplate();
        ~DriverTemplate();
        void spin();

    private:

        /* Class Variables */
        ros::NodeHandle nh;

        std::deque<robot_movement_interface::Command> command_list;
        std::deque<robot_movement_interface::Command> command_list_launched;

        static bool shutdown_requested;

        /* ROS Publishers, Subscribers and ServiceServers */
        ros::Subscriber sub_command_list;
        ros::Publisher pub_command_result;

        ros::ServiceServer srv_stop_robot;

        ros::Publisher pub_joint_states;
        ros::Publisher pub_tool_frame;
        ros::Publisher pub_robot_status;
        ros::Publisher pub_status;

        tf::TransformBroadcaster broadcaster;

        /* ROS callback functions */
        void callback_rmi_command(const robot_movement_interface::CommandListConstPtr &msg);
        bool callback_stop_robot(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        /* Basic driver functions */
        void connect_robot();
        void disconnect_robot();
        IndividualCommandTemplate process_command(robot_movement_interface::Command &command);
        void start_motion(IndividualCommandTemplate &indiv_command);
        void stop_motion();
        bool is_stopped();
        void halt_motion();
        bool in_error();
        robot_movement_interface::EulerFrame get_current_robot_pose();
        robot_movement_interface::EulerFrame get_current_waypoint();

        /* Helper functions */
        tf::Pose array_to_tf(std::vector<float> &array);
        tf::Pose euler_to_tf(robot_movement_interface::EulerFrame euler);
        bool equals(tf::Pose p1, tf::Pose p2, double limit_tlat = 0.0001, double limit_rot = 0.0001);

        /* Other functions */
        void publish();
        static void shutdown_signal(int signal);


};

#endif
