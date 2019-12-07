#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
//#include <tf/tf.h>
#include <tf/transform_listener.h>
laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;
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
        	"/base_link",
        	scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
		return;
  }
sensor_msgs::PointCloud cloud;
laser_geometry::LaserProjection projector_;
  		projector_.transformLaserScanToPointCloud("/link_chassis",*scan_in,cloud,listener_);
		pub_.publish(cloud);
}
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_and_publish");
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}