#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from visualization_msgs.msg import Marker
import numpy as np
import math

pub = None
repulsive_force = None
attractive_force = None
resultant_force = None
dist_precision_ = 0.3
position_ = Point()
yaw_ = 0
desired_position_ = Point()
desired_position_.x = 3
desired_position_.y = -1
desired_position_.z = 0
yaw_precision_ = math.pi / 40 # +/- 4.5 degree allowed
class MarkerBasics:
    def __init__(self):
	#red - sum of vectors
	#blue - repulsive force
	#green - attractive force
        self.marker_repulsive = rospy.Publisher('/marker_repulsive_force',Marker,queue_size =1)
	self.marker_attractive = rospy.Publisher('/marker_attractive_force',Marker,queue_size =1)
	self.marker_sum = rospy.Publisher('/marker_sum_of_forces',Marker,queue_size =1)
        self.rate = rospy.Rate(1)
        self.repulsive_marker(index = 0)
	self.attractive_marker(index = 0)
	self.sum_marker(index = 0)
    def repulsive_marker(self, index=0):
        global position_
	self.marker_object =[Marker(),Marker(),Marker()]
        self.marker_object[0] = Marker()
        self.marker_object[0].header.frame_id = '/odom'
        self.marker_object[0].header.stamp = rospy.get_rostime()
        self.marker_object[0].ns = "repulsive marker"
        self.marker_object[0].id  = index
        self.marker_object[0].type = Marker.ARROW
        self.marker_object[0].action = Marker.ADD
	start_point = Point()
        start_point.x = position_.x
        start_point.y = position_.y
        start_point.z = 0
	end_point = Point()
        end_point.x = repulsive_force[0] /100 
        end_point.y = repulsive_force[1] /100
        end_point.z = 0	
	self.marker_object[0].points = [start_point, end_point]
        self.marker_object[0].scale.x = 0.1
        self.marker_object[0].scale.y = 0.1
        self.marker_object[0].scale.z = 0.2
        self.marker_object[0].color.r =0
        self.marker_object[0].color.g =0
        self.marker_object[0].color.b =1
        self.marker_object[0].color.a =1
	self.marker_object[0].lifetime = rospy.Duration(0)
	self.marker_repulsive.publish(self.marker_object[0])
    def attractive_marker(self, index=0):
        global position_
        self.marker_object[1] = Marker()
        self.marker_object[1].header.frame_id = '/odom'
        self.marker_object[1].header.stamp = rospy.get_rostime()
        self.marker_object[1].ns = "attractive marker"
        self.marker_object[1].id  = index
        self.marker_object[1].type = Marker.ARROW
        self.marker_object[1].action = Marker.ADD
	start_point1 = Point()
        start_point1.x = position_.x
        start_point1.y = position_.y
        start_point1.z = 0
	end_point1 = Point()
        end_point1.x =attractive_force[0] *2
        end_point1.y = attractive_force[1] *2
        end_point1.z = 0	
	self.marker_object[1].points = [start_point1, end_point1]
        self.marker_object[1].scale.x = 0.1
        self.marker_object[1].scale.y = 0.1
        self.marker_object[1].scale.z = 0.2
        self.marker_object[1].color.r =0
        self.marker_object[1].color.g =0.5
        self.marker_object[1].color.b =0
        self.marker_object[1].color.a =1
	self.marker_object[1].lifetime = rospy.Duration(0)
	self.marker_attractive.publish(self.marker_object[1])
    def sum_marker(self, index=0):
        global position_
	global resultant_force
        self.marker_object[2] = Marker()
        self.marker_object[2].header.frame_id = '/odom'
        self.marker_object[2].header.stamp = rospy.get_rostime()
        self.marker_object[2].ns = "sum marker"
        self.marker_object[2].id  = index
        self.marker_object[2].type = Marker.ARROW
        self.marker_object[2].action = Marker.ADD
	start_point2 = Point()
        start_point2.x = position_.x
        start_point2.y = position_.y
        start_point2.z = 0
	end_point2 = Point()
	end_point2.x =resultant_force[0]
	end_point2.y = resultant_force[1]
        #end_point.x =attractive_force[0] *2 +repulsive_force[0] /100 
        #end_point.y = attractive_force[1] *2+ repulsive_force[1] /100 
        end_point2.z = 0	
	self.marker_object[2].points = [start_point2, end_point2]
        self.marker_object[2].scale.x = 0.1
        self.marker_object[2].scale.y = 0.1
        self.marker_object[2].scale.z = 0.2
        self.marker_object[2].color.r =1
        self.marker_object[2].color.g =0
        self.marker_object[2].color.b =0
        self.marker_object[2].color.a =1
	self.marker_object[2].lifetime = rospy.Duration(0)
	self.marker_sum.publish(self.marker_object[2])
class Server:
    def __init__(self):
        self.linear_x  = 0
        self.angular_z= 0
	self.msg1 = rospy.Subscriber('/key_vel', Twist, self.clbk_keyboard)
        self.msg2 = rospy.Subscriber('/mybot/laser/scan', LaserScan, self.clbk_laser)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.clbk_odom)
    def clbk_keyboard(self,msg1):
    	self.linear_x = msg1.linear.x
    	self.angular_z = msg1.angular.z
    def clbk_laser(self,msg2):
	global repulsive_force
	global attractive_force
	global resultant_force
	repulsive_force = np.array([0.0, 0.0])
        attractive_force = np.array([0.0, 0.0])
	resultant_force = np.array([0.0, 0.0])
	for i,val in enumerate(msg2.ranges):
	    if(not isinstance(val,float) or abs(val) > 15):
                continue
            dist = val
	    x = dist * np.cos(np.deg2rad(i*0.5))
	    y = dist * np.sin(np.deg2rad(i*0.5))
	    single_repulsive_force = np.array([x, y])
	    single_repulsive_force = np.true_divide(single_repulsive_force, dist ** 2)
	    repulsive_force += single_repulsive_force
	distance_attractive = math.sqrt((desired_position_.x - position_.x)**2 +(desired_position_.y - position_.y)**2)
	x_attractive = (desired_position_.x - position_.x) / distance_attractive
	y_attractive = (desired_position_.y - position_.y) / distance_attractive
	attractive_force = np.array([x_attractive, y_attractive])
	print("Repulsive force", np.true_divide(repulsive_force,100))
        print("Attractive force" , np.multiply(attractive_force,2))
	#print(np.multiply(attractive_force, 2),np.true_divide(repulsive_force,100) )
	resultant_force = np.add(np.multiply(attractive_force, 2), np.true_divide(repulsive_force,100)) 
	print("resultant force" , resultant_force)
        publish_markers = MarkerBasics()
    	self.take_action()
    def clbk_odom(self, msg):
	global position_
	global yaw_
	#position
	position_ = msg.pose.pose.position
	#print(position_)
	#yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	#print(euler, "hii")
	yaw_ = euler[2]
    def take_action(self):
	global yaw_, yaw_precision_,dist_precision_
	linear_x = 0 
    	angular_z = 0
	average_angular = 0.25
    	msg = Twist()
	desired_yaw = math.atan2(resultant_force[1], resultant_force[0]) #y / x
	err_pos = math.sqrt(pow(desired_position_.y - position_.y,2) + pow(desired_position_.x-position_.x,2))
	err_yaw = desired_yaw - yaw_
	#print (err_pos, dist_precision_)
	#print(math.fabs(err_yaw) > yaw_precision_)
	if err_pos < dist_precision_ :
	    print ("The robot reached the point",(desired_position_.x, desired_position_.y))
	    msg.linear.x = 0
    	    msg.angular.z = 0
    	    pub.publish(msg)
            rospy.signal_shutdown("robot reached destination")
	elif math.fabs(err_yaw) > yaw_precision_:
	    if err_yaw > 0 :
                angular_z = average_angular
            else:
		angular_z = -1 * average_angular
    	    #angular_z = desired_yaw if err_yaw >0 or math.fabs(yaw_ > 1) else -1 *desired_yaw
	elif err_pos > dist_precision_:
	    linear_x = 0.4
	    #angular_z = 0
	msg.linear.x = linear_x
    	msg.angular.z = angular_z
    	pub.publish(msg)
def main():
    global pub
    rospy.init_node('Obstacle_Avoidance')
    server = Server()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    
    rospy.spin()

if __name__ == '__main__':
    main()

