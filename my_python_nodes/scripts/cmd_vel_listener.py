#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def callback(msg: Twist):
    rospy.loginfo(f"[cmd_vel] linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")

def main():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber('/cmd_vel', Twist, callback)
    rospy.loginfo("cmd_vel listener started, waiting for messages…")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
