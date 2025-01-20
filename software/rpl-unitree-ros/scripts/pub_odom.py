#! /usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

NUM_INITIAL_TRIES = 10
WAIT_TIME_TRIES = 1
CUR_TRY = 0

rospy.init_node('odom_pub')

odom_pub=rospy.Publisher('/gazebo_odom', Odometry, queue_size=1)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id='/odom'

model = GetModelStateRequest()
model.model_name='robot'

r = rospy.Rate(200)

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

    r.sleep()