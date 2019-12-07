#!/usr/bin/env python
import rospy
import time
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
    v1 =-(-2* data.linear.x + width *3* data.angular.z)/2
    v2 = (2* data.linear.x + width *3* data.angular.z)/2
    if(abs(v1) < 0.01 and abs(v2) < 0.01):
        kit.motor3.throttle = 0
        kit.motor4.throttle = 0
        return
    while(abs(v1) < 0.18 and abs(v2) < 0.18):
        v1 = v1 * 2
        v2 = v2 * 2
    if((abs(v1)  > 0.11 and abs(v2)  > 0.11) and abs(v1)  < 0.13 and abs(v2)  < 0.13):
        v1 = v1 * 2
        v2 = v2 * 2
    if(v1 > 0.35):
        v1 =0.35
    elif (v1<-0.35):
        v1 = -0.35
    if(v2 > 0.35):
        v2 =0.35
    elif (v2<-0.35):
        v2 = -0.35
    print(v1,v2)
    if not v1 or not v2 or (abs(v1)<0.1 and abs(v2) <0.1) or (abs(v1) > 0.35 or abs(v2) > 0.35) :
        v1=0
        v2=0
    kit.motor3.throttle =   -1 * v1
    kit.motor4.throttle =   -1 * v2
    #time.sleep(1.0)
    #kit.motor1.throttle = 0
    #kit.motor2.throttle = 0
def random_subscriber():
    rospy.init_node("random_subscriber")
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()
if __name__ == '__main__':
    random_subscriber()
    rospy.on_shutdown(cleanup)

