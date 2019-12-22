#!/usr/bin/env python
import rospy
import time
import math
from adafruit_motorkit import MotorKit
from geometry_msgs.msg import Twist
kit = MotorKit()
def cleanup():
    kit.motor3.throttle=0
    kit.motor4.throttle=0
def callback(data):
    print("callback")
    rospy.loginfo("I receive %s", data)
    width = 0.15
    linear_velocity = data.linear.x
    angular_velocity = data.angular.z
    if(math.fabs(angular_velocity) > 1):
        angular_velocity= 1 if angular_velocity > 0 else -1
        data.angular.z= 1 if data.angular.z > 0 else -1
    if(math.fabs(linear_velocity) > 0.25):
        linear_velocity = 0.25 if linear_velocity  > 0 else -0.25
        data.linear.x= 0.25 if data.linear.x > 0 else -0.25
    v1 =-(-2* data.linear.x + width * data.angular.z)/2
    v2 = (2* data.linear.x + width * data.angular.z)/2
    print(v1,v2)
    velocity_list= [v1,v2]
    for i,item in enumerate(velocity_list):
        if(abs(velocity_list[i]) > 0.043 and abs(velocity_list[i]) < 0.13733):
            velocity_list[i] = ((abs(velocity_list[i])+0.330182)/1.87006) * abs(velocity_list[i])/velocity_list[i]
        elif(abs(velocity_list[i]) > 0.13733 and abs(velocity_list[i]) < 0.2575):
            velocity_list[i] = ((abs(velocity_list[i])+0.463502)/2.40334) * abs(velocity_list[i])/velocity_list[i]
        elif(abs(velocity_list[i]) > 0.2575 and abs(velocity_list[i]) < 0.412):
            velocity_list[i] = ((abs(velocity_list[i])+0.6695)/3.09) * abs(velocity_list[i])/velocity_list[i]
        else:
            print("Value is %s is out of range", data.linear.x)
            #cleanup()
            kit.motor3.throttle=0
            kit.motor4.throttle=0
            return
    v1=velocity_list[0]
    v2= velocity_list[1]
    print(v1,v2)
    if(data.linear.x is 0):    
        kit.motor3.throttle =    v1 if v1+0.5>v2 and  abs(v1) < 0.35 and abs(v1) >0.19 else 0
        kit.motor4.throttle =    v2 if v2+0.5>v1 and abs(v2) < 0.35 and abs(v2) > 0.19 else 0
    else:
        kit.motor3.throttle =   -1 * v1 if v1 and  abs(v1) < 0.35 and abs(v1) >0.19 else 0
        kit.motor4.throttle =   -1 * v2 if v2 and abs(v2) < 0.35 and abs(v2) > 0.19 else 0
    #time.sleep(1.0)
    #kit.motor1.throttle = 0
    #kit.motor2.throttle = 0
def random_subscriber():
    rospy.init_node("motor_velocity_controller")
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()
if __name__ == '__main__':
    random_subscriber()
    rospy.on_shutdown(cleanup)

