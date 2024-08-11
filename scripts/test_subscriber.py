#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan_data):
    rospy.loginfo("Test Callback executed.")
    rospy.loginfo("Scan data ranges: %s", scan_data.ranges)

def listener():
    rospy.init_node('test_listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
