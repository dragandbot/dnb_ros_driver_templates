import math
import tf
import rospy

from robot_movement_interface.msg import EulerFrame
from geometry_msgs.msg import Vector3, Pose, Quaternion, TransformStamped, PoseStamped

class RMILib():

    def __init__(self):
        pass

    # conversion functions:
    '''
    @brief eulerToPoseTF: function to convert a robot_movement_interface::EulerFrame into a geometry_msgs::Pose
    @param e
    @return
    '''
    @staticmethod
    def eulerToPoseGeo(euler_frame):
        v = Vector3(euler_frame.x, euler_frame.y, euler_frame.z)
        q = RMILib.createQuaternionFromRPY(euler_frame.gamma, euler_frame.beta, euler_frame.alpha)
        return Pose(v, q)

    '''
    @brief arrayToPoseTF: function to convert an array into a geometry_msgs::Pose
    @details The array is resized to match have the size of 6 elements.
    @param array to convert. The function expects the array to have the folowing orders "x, y, z, alpha, beta, gamma" or "x, y, z, roll, pitch, yaw" (ZYX-Euler-Intrinsic convention).
    @return
    '''
    @staticmethod
    def arrayToPoseGeo(array):
        while (len(array) < 6):
            array.append(0.0)
        v = Vector3(array[0], array[1], array[2])
        q = RMILib.createQuaternionFromRPY(array[5], array[4], array[3])
        return Pose(v, q)

    '''
    @brief xyzrpyToPoseTF: function to convert coordinates of a pose into a geometry_msgs::Pose
    @details The function uses the ZYX-Euler-Intrinsic convention for the rotation angles.
    @param x
    @param y
    @param z
    @param roll
    @param pitch
    @param yaw
    @return
    '''
    @staticmethod
    def xyzrpyToPoseGeo(x, y, z, roll, pitch, yaw):
        v = Vector3(x, y, z)
        q = RMILib.createQuaternionFromRPY(roll, pitch, yaw)
        return Pose(v, q)

    '''
    @brief xyzabcToPoseTF: function to convert coordinates of a pose into a geometry_msgs::Pose
    @details The function uses the ZYX-Euler-Intrinsic convention for the rotation angles.
    @param x
    @param y
    @param z
    @param alpha
    @param beta
    @param gamma
    @return
    '''
    @staticmethod
    def xyzabcToPoseGeo(x, y, z, alpha, beta, gamma):
        v = Vector3(x, y, z)
        q = RMILib.createQuaternionFromRPY(gamma, beta, alpha)
        return Pose(v, q)

    '''
    @brief poseTFToEuler: function to convert a given geometry_msgs::Pose into a robot_movement_interface::EulerFrame
    @details The function uses the ZYX-Euler-Intrinsic convention for the rotation angles.
    @param p
    @return
    '''
    @staticmethod
    def poseGeoToEuler(pose_geo):
        euler_frame = EulerFrame()
        euler_frame.x = pose_geo.position.x
        euler_frame.y = pose_geo.position.y
        euler_frame.z = pose_geo.position.z

        roll, pitch, yaw = RMILib.quaternionToRPY(pose_geo.orientation)
        euler_frame.alpha = yaw
        euler_frame.beta = pitch
        euler_frame.gamma = roll
        return euler_frame

    #helper functions:
    '''
    @brief difference: function to calculate the difference of between two poses
    @param p1
    @param p2
    @return p1 - p2 for translation and rotation
    '''
    @staticmethod
    def difference(pose1, pose2):
        pose_geo = Pose()
        pose_geo.position = RMILib.sub_vec(pose1.position, pose2.position)
        pose_geo.orientation = RMILib.sub_quat(pose1.orientation, pose2.orientation)
        return pose_geo

################################################################################
    # Vector3 helper functions
    @staticmethod
    def add_vec (v1, v2):
        v = Vector3()
        v.x = v1.x + v2.x
        v.y = v1.y + v2.y
        v.z = v1.z + v2.z
        return v

    @staticmethod
    def sub_vec (v1, v2):
        v = Vector3()
        v.x = v1.x - v2.x
        v.y = v1.y - v2.y
        v.z = v1.z - v2.z
        return v

    @staticmethod
    def length_vec(v):
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

################################################################################
    # Quaternion helper functions
    @staticmethod
    def add_quat (q1, q2):
        q = Quaternion()
        q.x = q1.x + q2.x
        q.y = q1.y + q2.y
        q.z = q1.z + q2.z
        q.w = q1.w + q2.w
        return q

    @staticmethod
    def sub_quat (q1, q2):
        q = Quaternion()
        q.x = q1.x - q2.x
        q.y = q1.y - q2.y
        q.z = q1.z - q2.z
        q.w = q1.w - q2.w
        return q

    @staticmethod
    def dot_quat(q1, q2):
        return (q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w)

    @staticmethod
    def length_quat(q):
        return math.sqrt(RMILib.dot_quat(q,q))

    @staticmethod
    def length2_quat(q):
        return RMILib.dot_quat(q,q)

    @staticmethod
    def negate_quat(q):
        return Quaternion(-q.x, -q.y, -q.z, -q.w)

    @staticmethod
    def angleShortestPath(q1, q2):
        s = math.sqrt(RMILib.length2_quat(q1) * RMILib.length2_quat(q2))
        # Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
        if (RMILib.dot_quat(q1, q2) < 0):
            q1_ = RMILib.negate_quat(q1)
        else:
            q1_ = q1
        return math.acos(max(min(RMILib.dot_quat(q1_, q2) / s, 1.0), -1.0)) * 2.0

    @staticmethod
    def createQuaternionFromRPY(roll, pitch, yaw):
        q = Quaternion()
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        q.x = quaternion[0]
        q.y = quaternion[1]
        q.z = quaternion[2]
        q.w = quaternion[3]
        return q

    @staticmethod
    def quaternionToRPY(q):
        quaternion = (q.x, q.y, q.z, q.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw

    @staticmethod
    def quaternionCommandToEuler(cmd):
        quaternion = (cmd.pose[3], cmd.pose[4], cmd.pose[5], cmd.pose[6])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        cmd.pose[3] = euler[2]
        cmd.pose[4] = euler[1]
        cmd.pose[5] = euler[0]
        cmd.pose = cmd.pose[:6] # resize the list to 6 items
        cmd.pose_type = "EULER_INTRINSIC_ZYX"
        return euler

    # Generates a PoseStamped object up from EulerFrame object
    @staticmethod
    def eulerFrameToPoseStamped(parentframename, eframe):
        pose = PoseStamped()

        quaternion = tf.transformations.quaternion_from_euler(eframe.gamma, eframe.beta, eframe.alpha, axes='sxyz')

        pose.header.frame_id = parentframename
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = eframe.x
        pose.pose.position.y = eframe.y
        pose.pose.position.z = eframe.z
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose


    # Generates a TransformStamped object up from PoseStamped object
    @staticmethod
    def poseStampedToTransformStamped(pose, childframename):
        t = TransformStamped()

        t.header.stamp = pose.header.stamp
        t.header.frame_id = pose.header.frame_id
        t.child_frame_id = childframename
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation.x = pose.pose.orientation.x
        t.transform.rotation.y = pose.pose.orientation.y
        t.transform.rotation.z = pose.pose.orientation.z
        t.transform.rotation.w = pose.pose.orientation.w
        return t