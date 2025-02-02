#! /usr/bin/env python3
import rospy

from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Header

def main():


    rospy.init_node('twist_stamped_to_twist')

    cmd_vel_pub = rospy.Publisher('/cmd_vel_twist', Twist, queue_size=1)

    def cmd_vel_cb(msg: TwistStamped):
        new_msg = Twist()
        new_msg.linear = msg.twist.linear
        new_msg.angular = msg.twist.angular
        cmd_vel_pub.publish(new_msg)


    cmd_vel_sub = rospy.Subscriber('/cmd_vel', TwistStamped, cmd_vel_cb)

    rospy.spin()

main()