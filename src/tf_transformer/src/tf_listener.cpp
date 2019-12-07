#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
/*
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<sensor_msgs::PointCloud>("/laserPointCloud", 1);
    sub_ = n_.subscribe("/scan", 1, &SubscribeAndPublish::callback, this);
  }

  void callback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
	if(!listener_.waitForTransform(
        	scan_in->header.frame_id,
        	"/link_chassis",
        	scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        	ros::Duration(1.0)))
        {
		return;
        }
  	
  
  	projector_.transformLaserScanToPointCloud("/link_chassis", *scan_in, cloud, listener_);
  	pub_.publish(cloud);
  }
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  sensor_msgs::PointCloud cloud;
};
*/


sensor_msgs::PointCloud cloud;
laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;
int transformFlag = 0;

void callback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
        if(!listener_.waitForTransform(
                scan_in->header.frame_id,
                "/link_chassis",
                scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                ros::Duration(1.0)))
        {
                return;
        }

        projector_.transformLaserScanToPointCloud("/link_chassis", *scan_in, cloud, listener_);
        transformFlag = 1;
  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_and_publish");
  //SubscribeAndPublish SAPObject;
 
  ros::NodeHandle n_;
  ros::Publisher pub_ = n_.advertise<sensor_msgs::PointCloud>("/laserPointCloud", 1);
  ros::Subscriber sub_ = n_.subscribe("/scan", 1, &callback);

  ros::Rate r(10);
  while (ros::ok())
  {
     if(transformFlag)
     {
        pub_.publish(cloud);
        transformFlag = 0;
     }
     ros::spinOnce();
     r.sleep();
  } 
  ros::spin();
  return 0;
}
