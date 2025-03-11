#! /usr/bin/env python3
import numpy as np

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import Header
import geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix

import tf2_ros

NUM_INITIAL_TRIES = 10
WAIT_TIME_TRIES = 1
CUR_TRY = 0

def transform_stamped_to_matrix(msg):
    mat = np.eye(4, dtype=np.float32)
    mat[0, 3] = msg.transform.translation.x
    mat[1, 3] = msg.transform.translation.y
    mat[2, 3] = msg.transform.translation.z
    quat_array = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
    mat[:3, :3] = quaternion_matrix(quat_array)[:3, :3]
    return mat
        

class OdomTransformer:
    def __init__(self):
        self.transform_parent = rospy.get_param('~transform_parent', 'map')
        self.transform_child = rospy.get_param('~transform_child', 'base')
        self.odom_pub = rospy.Publisher("/state_estimation", Odometry, queue_size=1)
        self.odom_sub = rospy.Subscriber("/Odometry", Odometry, self.pub_odom)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def pub_odom(self, msg):

        cur_world_id = msg.header.frame_id
        cur_child_id = msg.child_frame_id

        try:
            parent_transform = self.tfBuffer.lookup_transform(self.transform_parent, cur_world_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(f"couldn't get transform from {self.transform_parent} to {cur_world_id}")
            return

        try:
            child_transform = self.tfBuffer.lookup_transform(cur_child_id, self.transform_child, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(f"couldn't get transform from {cur_child_id} to {self.transform_child}")
            return
        
        quat_array = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        orig_transform = np.eye(4, dtype=np.float32)
        orig_transform[:3, :3] = quaternion_matrix(quat_array)[:3, :3]
        orig_transform[:3, 3] = [x, y, z]

        new_transform = transform_stamped_to_matrix(parent_transform) @ orig_transform @ transform_stamped_to_matrix(child_transform)


        odom=Odometry()
        header = Header()
        header.frame_id = self.transform_parent
        header.stamp = rospy.Time.now()
        odom.header = header
        odom.child_frame_id = self.transform_child

        pose = PoseWithCovariance()
        pose.covariance = msg.pose.covariance
        pose.pose.position.x = new_transform[0, 3]
        pose.pose.position.y = new_transform[1, 3]
        pose.pose.position.z = new_transform[2, 3]
        
        new_quat = quaternion_from_matrix(new_transform)
        pose.pose.orientation.x = new_quat[0]
        pose.pose.orientation.y = new_quat[1]
        pose.pose.orientation.z = new_quat[2]
        pose.pose.orientation.w = new_quat[3]

        odom.pose = pose
        
        self.odom_pub.publish(odom)


if __name__ == "__main__":

    rospy.init_node('odom_pub_from_topic')

    transformer = OdomTransformer()

    rospy.spin()
