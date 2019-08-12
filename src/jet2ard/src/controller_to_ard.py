#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    twist = Twist()
    twist.linear.x = data.axes[1]
#    print("running callback")
    twist.angular.z =  data.axes[2]
    pub.publish(twist)

def start():
    global pub
    pub = rospy.Publisher('jetsoncar_teleop_joystick/cmd_vel', Twist, queue_size = 1)
    rospy.Subscriber('joy', Joy, callback)
    rospy.init_node('Jet2Ard')
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.RosInterruptException:
        pass
