#! /usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import geometry_msgs
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import tf2_ros

NUM_INITIAL_TRIES = 10
WAIT_TIME_TRIES = 1
CUR_TRY = 0

rospy.init_node('odom_pub')


rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

model_name = rospy.get_param('~model_name', 'robot')
pub_rate = int(rospy.get_param('~pub_rate', 100))
out_frame_id = rospy.get_param('~out_frame_id', '/state_estimation')
transform_parent = rospy.get_param('~transform_parent', 'map')
transform_child = rospy.get_param('~transform_child', 'base')
odom_pub=rospy.Publisher(out_frame_id, Odometry, queue_size=1)

odom=Odometry()
header = Header()
header.frame_id = out_frame_id

model = GetModelStateRequest()
model.model_name = model_name

r = rospy.Rate(pub_rate)

br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = transform_parent
t.child_frame_id = transform_child

while not rospy.is_shutdown():

    result = get_model_srv(model)

    if not result.success:
        if CUR_TRY < NUM_INITIAL_TRIES:
            CUR_TRY += 1
            rospy.sleep(WAIT_TIME_TRIES)
            continue
        else:
            raise RuntimeError(f"can't get odom from gazebo:\n{str(result)}")

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish(odom)

    quat_array = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]


    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = odom.pose.pose.position.x
    t.transform.translation.y = odom.pose.pose.position.y
    t.transform.translation.z = odom.pose.pose.position.z
    t.transform.rotation.x = quat_array[0]
    t.transform.rotation.y = quat_array[1]
    t.transform.rotation.z = quat_array[2]
    t.transform.rotation.w = quat_array[3]
    euler_angles = euler_from_quaternion(quat_array)
    # rospy.loginfo(f"orientation: {str(euler_angles)}")

    # rospy.loginfo(f"time: {rospy.Time.now()}")
    br.sendTransform(t)

    r.sleep()