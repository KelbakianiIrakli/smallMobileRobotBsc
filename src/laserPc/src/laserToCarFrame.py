#!/usr/bin/env python
import rospy
import tf
import numpy as np
import math
from sensor_msgs.msg import PointCloud as pc
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
listener = tf.TransformListener()
class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPointCloud",pc, queue_size=1)
        self.laserSub = rospy.Subscriber("/scan",LaserScan, self.laserCallback,queue_size=1 )
    def laserCallback(self,data):
        #print(data)
        cloud2_out = self.laserProj.projectLaser(data)
        #print(cloud2_out)
        transformed_cloud = pc()
        transformed_cloud.header.seq = cloud2_out.header.seq
        transformed_cloud.header.stamp = cloud2_out.header.stamp
        transformed_cloud.header.frame_id = '/link_chassis'
        points_from_cloud2 = point_cloud2.read_points(cloud2_out)
        for p in points_from_cloud2:
            laser_point_msg = PointStamped()
            laser_point_msg.header.frame_id ='/laser_frame'
            laser_point_msg.header.stamp = cloud2_out.header.stamp
            laser_point_msg.header.seq = cloud2_out.header.seq
            laser_point_msg.point = Point(p[0], p[1], p[2])
            #print(laser_point_msg) #here it shows points
            #print(point)
            #print(type(laser_point_msg.point))
            #listener = tf.TransformListener()
            try:
                transformedPoint = listener.transformPoint('/link_chassis',laser_point_msg)
                #print(transformedPoint)
                #print(transformed_cloud)
            except(tf.LookupException,tf.ConnectivityException, tf.ExtrapolationException):
                continue
            transformed_cloud.points.append(transformedPoint.point)
        #listener.waitForTransform('/link_chassis', '/laser_frame',)
        #transformed_cloud.channels = cloud_out.channels
        #print(transformed_cloud)
        self.pcPub.publish(transformed_cloud)
if __name__ == '__main__':
    rospy.init_node("laserPointCloud")
    l2pc= Laser2PC()
    rospy.spin()