#! /usr/bin/env python
import numpy as np
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None
repulsive_force = None
state_description = ''
class Server:
    def __init__(self):
        self.linear_x  = 0.2
        self.angular_z= 0
        self.msg1 = rospy.Subscriber('/key', Twist, self.clbk_keyboard)
        self.msg2 = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
    def clbk_keyboard(self,msg1):
        self.linear_x = msg1.linear.x
        self.angular_z = msg1.angular.z
    def clbk_laser(self,msg2):
        global repulsive_force
        repulsive_force = np.array([0.0,0.0])
        count = int(msg2.scan_time/msg2.time_increment)
        print("I calculated repulsive force:")
        left,fleft,front,fright,right = ([] for k in range(5))
        for i in range(count):
            degree = math.degrees(msg2.angle_min + msg2.angle_increment * i)
            if abs(degree) > 90 or not isinstance(msg2.ranges[i],float) or abs(msg2.ranges[i]) > 8:
                continue
            if(degree >  0 and degree <36):
                right.append(msg2.ranges[i])
            elif( degree > 36 and degree <72):
                fright.append(msg2.ranges[i])
            elif( degree >72 and degree <108):
                front.append(msg2.ranges[i])
            elif( degree >108 and degree <144):
                fleft.append(msg2.ranges[i])
            elif( degree > 144 and degree <180):
                left.append(msg2.ranges[i])
            dist = msg2.ranges[i]
            x = dist * np.cos(np.deg2rad(degree))
            y = dist * np.sin(np.deg2rad(degree))
            #print(x,y)
            single_repulsive_force = np.array([x, y])
            #print(single_repulsive_force,dist)
            single_repulsive_force = np.true_divide(single_repulsive_force, dist**2)
            #print(single_repulsive_force)
            repulsive_force += single_repulsive_force
        print(repulsive_force)
    def take_action(self,regions):
	global state_description
    	linear_x = 0.4 
    	angular_z = 0
    	msg = Twist()
    	d = 0.4
    	if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing in front'
            linear_x = self.linear_x
            angular_z = self.angular_z
    	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d :
            state_description = 'case 2 - front'
            linear_x= self.linear_x/2
            if(self.angular_z > 0 and self.angular_z <0.3):
                angular_z = self.angular_z +0.4
	    else:
                angular_z = min(abs(0.5 - abs(self.angular_z))/2,0.4) #rotate left by 2 times less user intput
        	#self.linear_x  = 0
            self.angular_z= 0
    	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d :
            state_description = 'case 3 - fright'
            linear_x= self.linear_x/2
            if(self.angular_z > -0.3 and self.angular_z <0):
                angular_z = self.angular_z -0.4
	    else:
                angular_z = min(abs(0.4  - abs(self.angular_z))/2,0.4)*(-1) #rotate left by 2 times less user intput
     
        	#self.linear_x  = 0
            self.angular_z= 0
    	elif regions['front'] > d  and regions['fleft'] < d and regions['fright'] > d :
            state_description = 'case 4 - fleft'
            linear_x= self.linear_x/2
            if(self.angular_z > 0 and self.angular_z <0.4):
                angular_z = self.angular_z +0.4
	    else:
                angular_z = min(abs(0.4  - abs(self.angular_z))/2,0.4) #rotate left by 2 times less user intput
        	#self.linear_x  = 0
            self.angular_z= 0
	elif regions['front'] < d  and regions['fleft'] > d and regions['fright'] < d :
            state_description = 'case 5 - front and fright'
            linear_x= self.linear_x/2
            if(self.angular_z > -0.3 and self.angular_z <0):
                angular_z = self.angular_z -0.4
	    else:
		angular_z = min(abs(0.4  - abs(self.angular_z))/2,0.4)*(-1) #rotate left by 2 times less user intput
        	#self.linear_x  = 0
            self.angular_z= 0
    	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d :
            state_description = 'case 6 - front and fleft'
            linear_x= self.linear_x/2
            if(self.angular_z > 0 and self.angular_z <0.4):
                angular_z = self.angular_z +0.4
	    else:
                angular_z = min(abs(0.4  - abs(self.angular_z))/2,0.4) #rotate left by 2 times less user intput
        	#self.linear_x  = 0
            self.angular_z= 0
	elif regions['front'] < d  and regions['fleft'] < d and regions['fright'] < d :
            state_description = 'case 7 - front and fleft and fright'
            linear_x= self.linear_x/2
            angular_z = min(abs(0.7  - abs(self.angular_z))/2,0.5) #rotate left by 2 times less user intput
        	#self.linear_x  = 0
            self.angular_z= 0
    	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d :
            state_description = 'case 8 - fleft and fright'
            linear_x= self.linear_x
        	#angular_z = abs(self.angular_z)/2 #rotate left by 2 times less user intput
        	#self.linear_x  = 0
            self.angular_z= 0
    	else:
            state_description = 'unknown case'
            rospy.loginfo(regions)
        rospy.loginfo(state_description)
    	msg.linear.x = linear_x
    	msg.angular.z = angular_z
    	print(msg)
    	pub.publish(msg)

def main():
    global pub
    rospy.init_node('Obstacle_Avoidance')
    server = Server()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.spin()
    
    
if __name__ == '__main__':
    main()

