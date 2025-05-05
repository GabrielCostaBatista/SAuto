#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('simple_publisher')
    pub = rospy.Publisher('/chatter', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    count = 0
    while not rospy.is_shutdown():
        msg = String(data=f"hello {count}")
        pub.publish(msg)
        rospy.loginfo(f"Published: {msg.data}")
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
