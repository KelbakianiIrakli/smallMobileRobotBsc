#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import PointCloud as pc
from geometry_msgs.msg import Twist
pub = None
state_description = ''
class Server:
    def __init__(self):
        self.linear_x  = 0.22
        self.angular_z= 0
        self.msg1 = rospy.Subscriber('/key', Twist, self.clbk_keyboard)
        self.msg2 = rospy.Subscriber('/laserPointCloud', pc, self.clbk_pc)
    def clbk_keyboard(self,msg1):
        self.linear_x = msg1.linear.x
        self.angular_z = msg1.angular.z
    def clbk_pc(self,msg2):
        #count = 0
        #count = int(msg2.scan_time/msg2.time_increment)
        #print("I heard a laser scan:")
        left,fleft,front,fright,right = ([] for k in range(5))
        for i,val in enumerate(msg2.points):
            if(val.x < 0 and val.y <0 or val.x<0 and val.y > 0):
                degree = math.degrees(math.atan(val.y/val.x))
            #degree = math.degrees(msg2.angle_min + msg2.angle_increment * i)
                distance = math.sqrt(val.x**2 +val.y**2 )
                if(degree >  54 and degree < 90):
                    left.append(distance)
                elif( degree > 18 and degree <54):
                    fleft.append(distance)
                elif( degree >-18 and degree <18):
                    front.append(distance)
                elif( degree >-54 and degree <-18):
                    fright.append(distance)
                elif( degree > -90 and degree <-54):
                    right.append(distance)
                #print(distance)
                #array.append(distance)
                #print(len(array))
                #print(array)
            else:
                continue
        regions = {
            'left':  min(min(left), 100),
            	'fleft': min(min(fleft), 100),
            	'front':  min(min(front), 100),
            'fright': min(min(fright), 100),
            'right': min(min(right), 100),
            	}
        
        #print(regions)
        self.take_action(regions)
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
                angular_z = self.angular_z -0.3
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
            angular_z = min(abs(0.5  - abs(self.angular_z))/2,0.5) #rotate left by 2 times less user intput
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
    	#print(msg)
    	pub.publish(msg)

def main():
    global pub
    rospy.init_node('Obstacle_Avoidance')
    server = Server()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.spin()
    
    
if __name__ == '__main__':
    main()

