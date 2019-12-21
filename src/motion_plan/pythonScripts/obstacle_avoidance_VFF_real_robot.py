#! /usr/bin/env python
import numpy as np
import rospy
import math
from sensor_msgs.msg import PointCloud as pc
from geometry_msgs.msg import Twist
pub = None
repulsive_force = None
attractive_force = [-100,200]
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
        global repulsive_force
        global attractive_force 
        repulsive_force = np.array([0.0, 0.0])
        singular_repulsive_forces = []
        for i,val in enumerate(msg2.points):
            degree = math.degrees(math.atan(val.y/val.x))
            dist = math.sqrt(val.x**2 +val.y**2 )
            x = dist * np.cos(np.deg2rad(degree))
            y = dist * np.sin(np.deg2rad(degree))
            single_repulsive_force = np.array([x, y])
	    single_repulsive_force = np.true_divide(single_repulsive_force, dist ** 2)
	    repulsive_force += single_repulsive_force
        
        #print(regions)
        self.take_action()
	print(repulsive_force, repulsive_force+attractive_force )
    def take_action(self):
        msg = Twist()
        linear_x = 0
    	angular_z = 0
    	resultant_force = repulsive_force+attractive_force
    	linear_x = resultant_force[0] /100
    	angular_z = resultant_force[0] /20
    	msg.linear.x = linear_x
    	msg.angular.z = angular_z
    	print(msg)
    	pub.publish(msg)

def main():
    global pub
    rospy.init_node('Obstacle_Avoidance_for_VFF_real_robot')
    server = Server()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.spin()
    
    
if __name__ == '__main__':
    main()

