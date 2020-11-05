#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import thread

from robot_movement_interface.msg import *
from robot_movement_interface.srv import *
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from industrial_msgs.msg import RobotStatus
from dnb_msgs.msg import ComponentStatus

from rmi_lib import RMILib
import individual_command_template as ict



class DriverTemplate():
    def __init__(self):
        self.command_list = []
        self.command_list_launched = []
        self.lock_commandlist = thread.allocate_lock()

        self.sub_command_list = rospy.Subscriber('command_list', CommandList, self.callback_rmi_command)
        self.pub_command_result = rospy.Publisher('command_result', Result, queue_size = 1)

        self.srv_stop_robot = rospy.Service('stop_robot_right_now', Trigger, self.callback_stop_robot)
    
        self.pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size = 1)
        self.pub_tool_frame = rospy.Publisher('tool_frame', EulerFrame, queue_size = 1)
        self.pub_robot_status = rospy.Publisher('robot_status', RobotStatus, queue_size = 1)

        self.pub_status = rospy.Publisher('~status', ComponentStatus, queue_size=1, latch=True)

        self.broadcaster = tf.TransformBroadcaster()

        rospy.loginfo("[driver] diver initialized")

    def spin(self):
        loop_rate = rospy.Rate(100)

        cm_status = ComponentStatus()
        cm_status.status_id = ComponentStatus().RUNNING
        cm_status.status_msg = "Running"
        self.pub_status.publish(cm_status)

        while not rospy.is_shutdown():
            with self.lock_commandlist:
                # launch new commands
                if len(self.command_list) > 0:
                    # process the new movement command
                    indiv_command = self.process_command(self.command_list[0])

                    # start the new movement command
                    self.start_motion(indiv_command)

                    # move the command from the open list to the launched list
                    self.command_list_launched.append(self.command_list.pop(0))

            if len(self.command_list_launched) > 0:
                with self.lock_commandlist:

                    result_msg = Result()
                    result_msg.command_id = self.command_list_launched[-1].command_id

                    # check whether the current waypoint was reached
                    # <!> disable this function if your controller does not provide the current target waypoint pose (see get_current_waypoint() function below for further information)
                    # <disable_begin>
                    if not self.equals(RMILib.arrayToPoseGeo(self.command_list_launched[0].pose), RMILib.eulerToPoseGeo(self.get_current_waypoint())):
                        self.command_list_launched.pop(0)
                    # <disable_end>

                    # check, whether the robot is stopped. This indicates, whether the current trajectory has been completed.
                    if self.is_stopped():
                        # check if the robot was correctly finished. If the commands finished correctly, then result_code is 0
                        # if not finishes correctly, result_code must be -1
                        if self.in_error():
                            result_msg.result_code = -1
                        else:
                            result_msg.result_code = 0
                        self.pub_command_result.publish(result_msg)

                        self.command_list_launched = []

            self.publish()
            loop_rate.sleep()

        cm_status.status_id = ComponentStatus().STOPPED
        cm_status.status_msg = "Stopped"
        self.pub_status.publish(cm_status)

    def callback_rmi_command(self, msg):
        rospy.loginfo("[driver] rmi request received")

        with self.lock_commandlist:
            # if we replace the commands discard all movement commands and stop the robot first
            if (msg.replace_previous_commands):
                self.command_list = []
                self.command_list_launched = []

            # stop robot and delete all pending movement commands; it is not necessary to speed the robot down to zero
            self.stop_motion()

            # build new command list by appending it to the current commands
            for cmd in msg.commands:
                self.command_list.append(cmd)

    def callback_stop_robot(self, req):
        rospy.loginfo("[driver] stop robot request received")

        with self.lock_commandlist:
            # cancel current command list
            self.command_list = []
            self.command_list_launched = []

            #stop robot and delete all pending movement commands; it is absolutely necessary to speed the robot down to zero
            self.halt_motion()

        return TriggerResponse(success=True, message="")

    def publish(self):
        # Joint states
        jointState = JointState()
        jointState.header.stamp = rospy.get_rostime()

        # <!> Fill here the current values of all joints (here in pseudo code)
        #
        # for (j in JOINTS):
        #      jointState.name.append("JOINT_NAME_j")
        #      jointState.position.append(JOINT_POSE_j)
        #      jointState.velocity.append(JOINT_VELOCITY_j)
        #      jointState.effort.append(JOINT_EFFORT_j)

        if len(jointState.position) > 0 and len(jointState.velocity) > 0:
            self.pub_joint_states.publish(jointState)

        # Tool frame
        tool_frame = self.get_current_robot_pose()
        self.pub_tool_frame.publish(tool_frame)

        # tf
        p, r = self.euler_to_transform(tool_frame)
        self.broadcaster.sendTransform(p, r, rospy.get_rostime(), "robot_state_tcp", "manufacturer_base")

        # Robot status
        status = RobotStatus()
        status.header.stamp = rospy.get_rostime()

        # <!> Fill here the current robot mode (here in pseudo code)
        status.mode.val = 2# MODE # UNKNOWN = -1, MANUAL = 1, AUTO = 2; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/RobotMode.msg
        status.e_stopped.val = 0# E_STOPPED # UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
        status.drives_powered.val = 1# DRIVED_POWERED # UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
        status.motion_possible.val = 1# MOTION_POSSIBLE # UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
        status.in_motion.val = 0# IN_MOTION # UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
        status.in_error.val = 0# IN_ERROR # UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
        self.pub_robot_status.publish(status)

    def connect_robot(self):
        # <!> Put here your code to connect the robot to the driver
        rospy.loginfo("[driver] robot connected")

    def disconnect_robot(self):
        # <!> Put here your code to disconnect the robot from the driver
        rospy.loginfo("[driver] robot disconnected")

    def process_command(self, rmi_command):
        indiv_command = ict.IndividualCommandTemplate()
        # <!> Adjust here the general movement command to your individual movement controller command
        return indiv_command

    def start_motion(self, indiv_command):
        # <!> Put here your code to execute a not-already running individual movement command.
        # This function should just send the command to the robot controller, where it is queued with the others and processed one after another
        # The function is expected to be non-blocking during the movement.
        pass

    def stop_motion(self):
        # <!> Put here your code to stop all pending movement commands in the robot controller.
        # The robot doesn't have to be at standstill, when leaving this function.
        pass

    def is_stopped(self):
        # <!> Put here your code to check, whether the robot is stoppped or not.
        # The controller is then supposed to have no more pending movement commands.
        # The robot is then supposed to have it's velocities near to zero.
        pass

    def halt_motion(self):
        # <!> Modify this function to stop the robot to standstill if necessary
        # All pending movement commands should be discarded in the robot controller.
        # The robot should be at standstill, when leaving this function.
        while not self.is_stopped():
            self.stop_motion()

    def in_error(self):
        # <!> This function needs to return if robot is in error state or not
        # This can be implemented through a global state variable which is updated after each status message
        pass

    def get_current_robot_pose(self):
        current_pose = EulerFrame()
        # <!> Put here the current robot pose (tool_pose in world coordiantes) sent by the controller
        current_pose.x = 0.5
        current_pose.y = 0.0
        current_pose.z = 0.5
        current_pose.alpha = 0.0
        current_pose.beta = 0.0
        current_pose.gamma = 0.0
        return current_pose

    def get_current_waypoint(self):
        current_waypoint = EulerFrame()
        # <!> Put here the current waypoint pose the robot currently drives to (tool_pose in world coordiantes) sent by the controller.
        # All waypoints of a trajectory are sent to the robot controller with start_motion commands and then processed one after another.
        # When the robot processes the trajectory, it steps through all waypoints. The robot discards the current waypoint and continues with the next one, when he reached the first waypoint's position.
        # If your controller does not provide the current waypoint, the driver can't pause the motion and resume it later with the last reached waypoint.
        # In this case leave the function empty or delete it and comment or remove the 'reachedWaypointByChangingTarget' and 'restart_requested' section in the 'spin()' function.
        return current_waypoint

    def euler_to_transform(self, euler):
        q = tf.transformations.quaternion_from_euler(euler.gamma, euler.beta, euler.alpha, axes='sxyz')
        return (euler.x, euler.y, euler.z), (q[0], q[1], q[2], q[3])

    def equals(self, pose1, pose2, limit_tlat=0.0001, limit_rot=0.0001):
        diff_vec_tlat = RMILib.sub_vec(pose1.position, pose2.position)
        delta_tlat = RMILib.length_vec(diff_vec_tlat)
        delta_rot = RMILib.angleShortestPath(pose1.orientation, pose2.orientation)
        return delta_tlat < limit_tlat and delta_rot < limit_rot


if __name__ == '__main__':
    rospy.init_node('dnb_driver_template_python')    

    driver = DriverTemplate()
    
    driver.connect_robot()
    driver.spin()
    driver.disconnect_robot()
    rospy.loginfo("[driver] diver shutdown")
