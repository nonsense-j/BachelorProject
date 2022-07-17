#!/usr/bin/env python

from distutils.log import info
import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import sqrt

x = 0.0
y = 0.0 
flag = True

def newPose(msg):
    global x
    global y
    global theta

    x = msg.pose.position.x
    y = msg.pose.position.y

def dest_callback(dest):
    global flag
    flag = True
    print(f"destination is set at ({dest.x},{dest.y})")
    writePose(str(x) + "#" + str(y))
    while flag and not rospy.is_shutdown() :
        print(f"from ({x}, {y}) to ({dest.x}, {dest.y})")
        inc_x = dest.x -x
        inc_y = dest.y -y

        dist = sqrt(inc_x**2 + inc_y**2)

        if inc_x**2 + inc_y**2 > 0.01:
            speed.linear.x = 0.3 * inc_x / dist
            speed.linear.y = 0.3 * inc_y / dist
            print(f"speed is set as ({speed.linear.x},{speed.linear.y})")
        else:
            print(f"reach ({x},{y})")
            flag = False
            speed.linear.x = 0.0
            speed.linear.y = 0.0
            pub.publish(speed)
            time.sleep(1)
            writePose(str(dest.x) + "#" + str(dest.y))
        pub.publish(speed)
        r.sleep()

def stop_callback(msg): 
    global flag
    print(f"!!stop at ({x},{y})")
    flag = False
    speed.linear.x = 0.0
    speed.linear.y = 0.0
    pub.publish(speed)

def writePose(flag):
    with open('/home/nonsense/Projects/FinalTask/fprime/Ref/ResCapActuators/Flags/moveFlag.txt', 'w') as f:
        f.write(str(flag))


rospy.init_node("rescap_controller")

move_sub = rospy.Subscriber('/ResCap/move_dest', Point, dest_callback)
sub = rospy.Subscriber("/ResCap/ground_truth_to_tf/pose", PoseStamped, newPose)
stop_sub = rospy.Subscriber('/ResCap/move_stop', Empty, stop_callback)
pub = rospy.Publisher("/ResCap/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

print("Ready to get dest......")
rospy.spin()