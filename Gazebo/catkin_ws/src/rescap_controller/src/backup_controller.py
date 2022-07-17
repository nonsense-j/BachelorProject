#!/usr/bin/env python

from distutils.log import info
import rospy
import time
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import atan2, e

x = 0.0
y = 0.0 
theta = 0.0
flag = True

def newPose(msg):
    global x
    global y
    global theta

    x = msg.pose.position.x
    y = msg.pose.position.y

    rot_q = msg.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def dest_callback(dest):
    global flag
    flag = True
    first_toward = True
    print(f"destination is set at ({dest.x},{dest.y})")
    while flag and (not rospy.is_shutdown()) :
        print(f"from ({x}, {y}) to ({dest.x}, {dest.y})")
        inc_x = dest.x -x
        inc_y = dest.y -y

        angle_to_goal = atan2(inc_y, inc_x)

        if abs(angle_to_goal - theta) > 0.1:
            first_toward = True
            speed.linear.x = 0.0
            speed.angular.z = 0.25*(abs(angle_to_goal - theta)-0.1)/5.8+0.05
            print(f"angle:{abs(angle_to_goal - theta)} ====== angular speed:{speed.angular.z}")
        elif inc_x**2 + inc_y**2 > 0.04:
            if first_toward:
                first_toward = False
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                pub.publish(speed)
                time.sleep(1.5)
            speed.linear.x = 0.2
            speed.angular.z = 0.0
        else:
            print(f"reach ({x},{y})")
            flag = False
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        pub.publish(speed)
        r.sleep()

def stop_callback(msg): 
    global flag
    print(f"!!stop at ({x},{y})")
    flag = False
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)


rospy.init_node("rescap_controller")

move_sub = rospy.Subscriber('/ResCap/move_dest', Point, dest_callback)
sub = rospy.Subscriber("/ResCap/ground_truth_to_tf/pose", PoseStamped, newPose)
stop_sub = rospy.Subscriber('/ResCap/move_stop', Empty, stop_callback)
pub = rospy.Publisher("/ResCap/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

print("Ready to get dest......")
rospy.spin()