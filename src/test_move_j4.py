#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

desired_pos = 0.0
current_pos = 0.0
pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10)

def joint4_move():
    sub = rospy.Subscriber('joint4_move_subscriber', Float64, publish_data)
    sub_current_state = rospy.Subscriber('/robot/joint4_position_controller/state', Float64, store_data)
    
    rospy.init_node('Merey_joint4_move', anonymous=True)
    rospy.loginfo("Current joint position:" + str(current_pos))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutdown')


def store_data(values):
    current_pos = values.data


def publish_data(values):
    if(values.data > current_pos):
        desired_pos = values.data
        rospy.loginfo("Changing to desired position: " + str(desired_pos))
        pub.publish(desired_pos)
    else:
	rospy.loginfo("Current position is greater than desired")

if __name__ == '__main__':
    try:
        joint4_move()
    except rospy.ROSInterruptException:
        pass





