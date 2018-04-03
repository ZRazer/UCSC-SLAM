// Example code @ http://wiki.ros.org/pcl/Tutorials

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <cmath>
#include <Eigen>

class slam
{
public:
	slam()
	{
  		// Create a ROS subscriber for the input point cloud
		sub = nh.subscribe ("/marker_data", 100, &slam::marker_cb, this);

  		// Create a ROS publisher for giving pose estimate of robot and markers
		marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
		ROS_INFO("Initialized cloud_parse object...");
	}

  // Callback function upon recieving point cloud
	void marker_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		
	}

private:
	ros::Publisher marker_pub;
	ros::Subscriber sub;
	ros::NodeHandle nh; 

};//End of class cloud_cb_pub

int main (int argc, char** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "slam_node");
	slam slam_obj;

	// Spinning handles ROS events
	ros::spin();
	return 0;
}