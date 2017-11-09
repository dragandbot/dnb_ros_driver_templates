#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import thread

from robot_movement_interface.msg import *
from robot_movement_interface.srv import *
from geometry_msgs.msg import Pose, Vector3, Quaternion
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from industrial_msgs.msg import RobotStatus
from collections import deque
from dnb_rmi_library.rmi_lib import RMILib

import individual_command_template as ict

command_list = deque()
command_list_launched = deque()
restart_requested = False
lock_commandlist = thread.allocate_lock()
speed_factor = 1.0

def spin():
    global command_list
    global command_list_launched
    global restart_requested
    global pub_command_result

    loop_rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if (len(command_list) > 0):
            with lock_commandlist:
                # process the new movement command
                indiv_command = processCommand(command_list[0])

                # start the new movement command
                start_motion(indiv_command)

                # move the command from the open list to the launched list
                command_list_launched.append(command_list.popleft())

        if (len(command_list_launched) > 0):
            with lock_commandlist:
                # check whether the current waypoint was reached
                # <!> disable this function if your controller does not provide the current target waypoint pose (see get_current_waypoint() function below for further information)
                # <disable_begin>
                if (RMILib.reachedWaypointByChangingTarget(RMILib.arrayToPoseGeo(command_list_launched[0].pose), get_current_waypoint())):
                    command_list_launched.popfront()
                # <disable_end>

                # check, whether the robot is stopped. This indicates, whether the current trajectory has been completed.
                if (is_stopped()):
                    command_list_launched.clear()

                result_msg = Result()
                result_msg.command_id = command_list_launched[-1].command_id
                # check if the robot was correctly finished. If the commands finished correctly, then result_code is 0
                # if not finishes correctly, result_code must be -1
                if (in_error()):
                    result_msg.result_code = -1
                else:
                    result_msg.result_code = 0
                pub_command_result.publish(result_msg)

                # check for a restart
                # <!> disable this function if your controller does not provide the current target waypoint pose (see get_current_waypoint() function below for further information)
                # <disable_begin>
                if (restart_requested):
                    # stop the currently pending movement commands; it is not necessary to speed the robot down to zero
                    stop_motion()

                    # restart all already launched and not still completed movement commands
                    for cmd in command_list_launched:
                        # process the movement command
                        indiv_command = processCommand(cmd)

                        # start the movement command
                        start_motion(indiv_command)
                        restart_requested = False
                # <disable_end>

        publish()
        loop_rate.sleep()

def callback_rmi_command(msg):
    global command_list
    global command_list_launched
    rospy.loginfo("[driver] rmi request received")

    with lock_commandlist:
        # if we replace the commands discard all movement commands and stop the robot first
        if (msg.replace_previous_commands):
            command_list.clear()
            command_list_launched.clear()

        # stop robot and delete all pending movement commands; it is not necessary to speed the robot down to zero
        stop_motion()

        # build new command list by appending it to the current commands
        for cmd in msg.commands:
            command_list.append(cmd)

def callback_get_io(req):
    # <!> Get here your io's and write it to the service message (here in pseudo code)
    # res.value = IO[req.number]
    return GetIOResponse()

def callback_set_io(req):
    # <!> Set here your io's from the service message (here in pseudo code)
    # IO[req.number] = req.value
    return SetIOResponse()

def callback_stop_robot(req):
    global command_list
    global command_list_launched
    rospy.loginfo("[driver] stop robot request received")

    with lock_commandlist:
        # cancel current command list
        command_list.clear()
        command_list_launched.clear()

        #stop robot and delete all pending movement commands; it is absolutely necessary to speed the robot down to zero
        halt_motion()

    return TriggerResponse(True, "")

def callback_scale_speed(req):
    global speed_factor
    global restart_requested
    rospy.logdebug("[driver] Scale speed=%.2f%% request received", 100 * req.value)
    if ((req.value < 0.0) or (req.value > 1.0)):
        rospy.logwarn("[driver] Scale speed=%.2f%% is out of bounds and will be cropped to [1%%, 100%%]!", 100 * req.value)
        speed_factor = min(max(0.01, req.value), 1.0)
    elif (req.value == 0.0):
        rospy.logwarn("[driver] Scale speed=%.2f%% is out of bounds. Robot stops and cancels movement queue!", 100 * req.value)

        # stop the currently pending movement commands; it is not necessary to speed the robot down to zero
        stop_motion()

        return SetFloatResponse()
    else:
        speed_factor = req.value
    rospy.loginfo("[driver] Set speed to %i%%", 100 * speed_factor)

    restart_requested = True

    return SetFloatResponse()

def publish():
    global pub_joint_states
    global pub_tool_frame
    global pub_robot_status
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

    if (len(jointState.position) > 0 and len(jointState.velocity) > 0):
        pub_joint_states.publish(jointState)

    # Tool frame
    tool_frame = RMILib.poseGeoToEuler(get_current_robot_pose())
    pub_tool_frame.publish(tool_frame)

    # Robot status
    status = RobotStatus()
    status.header.stamp = rospy.get_rostime()

    # <!> Fill here the current robot mode (here in pseudo code)
    # status.mode.val = MODE # UNKNOWN = -1, MANUAL = 1, AUTO = 2; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/RobotMode.msg
    # status.e_stopped.val = E_STOPPED # UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
    # status.drives_powered.val = DRIVED_POWERED # UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
    # status.motion_possible.val = MOTION_POSSIBLE # UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
    # status.in_motion.val = IN_MOTION # UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
    # status.in_error.val = IN_ERROR # UNKNOWN = -1, TRUE = ON = ENABLED = HIGH = CLOSED = 1, FALSE = OFF = DISABLED = LOW = OPEN = 0; see ROS: industrial_msgs/RobotStatus.msg, industrial_msgs/TriState
    pub_robot_status.publish(status)

def connectRobot():
    # <!> Put here your code to connect the robot to the driver
    rospy.loginfo("[driver] robot connected")
    pass

def disconnectRobot():
    # <!> Put here your code to disconnect the robot from the driver
    rospy.loginfo("[driver] robot disconnected")
    pass

def processCommand(rmi_command):
    indiv_command = ict.IndividualCommandTemplate()
    # <!> Adjust here the general movement command to your individual movement controller command
    return indiv_command

def start_motion(indiv_command):
    # <!> Put here your code to execute a not-already running individual movement command.
    # This function should just send the command to the robot controller, where it is queued with the others and processed one after another
    # The function is expected to be non-blocking during the movement.
    pass

def stop_motion():
    # <!> Put here your code to stop all pending movement commands in the robot controller.
    # The robot doesn't have to be at standstill, when leaving this function.
    pass

def is_stopped():
    # <!> Put here your code to check, whether the robot is stoppped or not.
    # The controller is then supposed to have no more pending movement commands.
    # The robot is then supposed to have it's velocities near to zero.
    pass

def halt_motion():
    # <!> Modify this function to stop the robot to standstill if necessary
    # All pending movement commands should be discarded in the robot controller.
    # The robot should be at standstill, when leaving this function.
    while not is_stopped():
        stop_motion()

def in_error():
    # <!> This function needs to return if robot is in error state or not
    # This can be implemented through a global state variable which is updated after each status message
    pass

def get_current_robot_pose():
    current_robot_pose = Pose()
    # <!> Put here the current robot pose (tool_pose in world coordiantes) sent by the controller
    return current_robot_pose

def get_current_waypoint():
    current_waypoint_pose = Pose()
    # <!> Put here the current waypoint pose the robot currently drives to (tool_pose in world coordiantes) sent by the controller.
    # All waypoints of a trajectory are sent to the robot controller with start_motion commands and then processed one after another.
    # When the robot processes the trajectory, it steps through all waypoints. The robot discards the current waypoint and continues with the next one, when he reached the first waypoint's position.
    # If your controller does not provide the current waypoint, the driver can't pause the motion and resume it later with the last reached waypoint.
    # In this case leave the function empty or delete it and comment or remove the 'reachedWaypointByChangingTarget' and 'restart_requested' section in the 'spin()' function.
    return current_waypoint_pose

if __name__ == '__main__':
    global pub_command_result
    global pub_joint_states
    global pub_tool_frame
    global pub_robot_status

    rospy.init_node('dnb_driver_template_python')
    sub_command_list = rospy.Subscriber('command_list', CommandList, callback_rmi_command)
    pub_command_result = rospy.Publisher('command_result', Result, queue_size = 1)
    srv_get_io = rospy.Service('get_io', GetIO, callback_get_io)
    srv_set_io = rospy.Service('set_io', SetIO, callback_set_io)
    srv_scale_speed = rospy.Service('scale_speed', SetFloat, callback_scale_speed)
    srv_stop_robot = rospy.Service('stop_robot_right_now', Trigger, callback_stop_robot)
    pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size = 1)
    pub_tool_frame = rospy.Publisher('tool_frame', EulerFrame, queue_size = 1)
    pub_robot_status = rospy.Publisher('robot_status', RobotStatus, queue_size = 1)

    rospy.loginfo("[driver] diver initialized")
    connectRobot()
    spin()
    disconnectRobot()
    rospy.loginfo("[driver] diver shutdown")
