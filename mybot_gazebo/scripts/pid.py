#!/usr/bin/env python
import roslib
import rospy
import math  

from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String

counter = 0
err_ant = 0
errx_ant = 0
erry_ant = 0
dt = 0.8
v_x = 0
v_y = 0
w = 0


def Position(odom_data):
    global roll, pitch, yaw
    global err_ant, errx_ant,erry_ant
    global msg, v_x, v_y, w
    global counter
    curr_time = odom_data.header.stamp
    pose = odom_data.pose.pose #  the x,y,z pose and quaternion orientation
    counter= counter+1
    # getting the curretn position
    x_curr = pose.position.x
    y_curr = pose.position.y
    u2 = y_des - y_curr
    u1 = x_des - x_curr
    # transformating Quaternion to euler angles
    orientation_q = pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # calculating the law control for differential drive robot
    phi_des = math.atan(u2/u1)

    err_p = phi_des - yaw
    err_x = x_des - x_curr
    err_y = y_des - y_curr
    w = 4*(err_p)  + 0.04*(err_p + err_ant)*dt
    v_x = 2*(err_x) 
    v_y = 1*(err_y) 
    v = math.sqrt(u1*u1 + u2*u2)
    #v_x = v*math.cos(phi_des)
    #v_y = v*math.sin(phi_des)
    err_ant = err_p
    errx_ant = err_x
    erry_ant = err_y
    rospy.loginfo("X current position " + str(x_curr))
    rospy.loginfo("Y current position " + str(y_curr))

def timer_callback(event):
        global msg, v_x, v_y, w
        msg = Twist()
        pub_ = rospy.Publisher('/mybot/cmd_vel', Twist)
        msg.linear.x = v_x
        msg.linear.y = v_y
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = w
        speed = 0.4 
        pub_.publish(msg)

def begin():
    
    
    while not rospy.is_shutdown():
        rospy.init_node('oodometry', anonymous=True) #make node 

        rospy.Subscriber('/mybot/odom_diffdrive',Odometry,Position)
        timer = rospy.Timer(rospy.Duration(0.5), timer_callback)
        rospy.spin()    
        timer.shutdown()

     
if __name__ == "__main__":
        x_des = 5
        y_des = 5
        begin()

