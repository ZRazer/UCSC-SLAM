#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <cmath>
#include <slam_node/landmark.h>
#include <slam_node/lm_array.h>

#define PI 3.14159265
// define RVIZ for rviz visualization of features, otherwise normal operation with slam
//#define RVIZ 

struct Point
{
	double _x, _y;
};

// Struct for lines
// Lines stored as y = mx + b, _slope = m, _yInt = b
// struct stores min/max x and y values of points in set, as well 
// as outlier point and its distance R to the line

struct Line 
{
	double _slope, _yInt, xmin, xmax, xmean, ymean, ymin, ymax, out_x, out_y, R, xvar, yvar;
	double radius, angle;
	unsigned long nPoints;
	bool vline;
	double getYforX(double x) {
		if(vline)
		{
			ROS_DEBUG("Called getYforX for vline");
		}
		else
		{
			return _slope*x + _yInt;
		}	
	}
  // Construct line from points
	bool fitPoints(std::vector<Point> & points) 
	{
		nPoints = points.size();

		if( nPoints < 2 ) 
		{
        // Infinite lines fit a single point 
			return false;
		}
		double sumX=0, sumY=0, sumXY=0, sumX2=0, sumY2=0, tempXmin = 1000, tempXmax = -1000, tempYmin = 1000, tempYmax = -1000;
		for(int i=0; i<nPoints; i++) 
		{
			// Find min/max x and y coords
			if (points[i]._x < tempXmin)
				tempXmin = points[i]._x;
			if (points[i]._x > tempXmax)
				tempXmax = points[i]._x;
			if (points[i]._y < tempYmin)
				tempYmin = points[i]._y;
			if (points[i]._y > tempYmax)
				tempYmax = points[i]._y;

			// Calc sum and sum of squares
			sumX += points[i]._x;
			sumY += points[i]._y;
			sumXY += points[i]._x * points[i]._y;
			sumX2 += points[i]._x * points[i]._x;
			sumY2 += points[i]._y * points[i]._y;
		}
		// Set range of vals
		xmin = tempXmin;
		xmax = tempXmax;
		ymin = tempYmin;
		ymax = tempYmax;

		// prep for finding LSRL
		xmean = sumX / nPoints;
		ymean = sumY / nPoints;

		// Calc xvar and yvar
		xvar = (sumX2/nPoints) - (xmean*xmean);
		yvar = (sumY2/nPoints) - (ymean*ymean);

		double denominator = (nPoints*sumX2) - (sumX*sumX);
		double delta, temp = 0;

		// Check for vertical line
		if( std::fabs(denominator) < 1e-7 ) 
		{
			vline = true;
	    	// Find outlier
			for(int i=0; i<nPoints; i++) 
			{
				delta = points[i]._x - xmean;
				if(std::fabs(delta) > std::fabs(temp))
				{
					temp = delta;
					out_x = points[i]._x;
					out_y = points[i]._y; 
				}
			}
			angle = PI/2;
			radius = (ymax - ymin)/2;
			R = temp;
			ROS_DEBUG("Found line: x = %f , R val: %f , angle: %f, radius: %f", xmean,R,angle,radius);
		}
		else 
		{
			vline = false;
			_slope = ((nPoints*sumXY) - (sumX*sumY)) / denominator;
			_yInt = (sumY - (_slope*sumX))/nPoints;
			// Find outlier
			for(int i=0; i<nPoints; i++) 
			{
				// distance from point to line using dot product
				delta = std::fabs(points[i]._y - (points[i]._x * _slope) - _yInt) / sqrt(1 + _slope*_slope);
				if(std::fabs(delta) > std::fabs(temp))
				{
					temp = delta;
					out_x = points[i]._x;
					out_y = points[i]._y;
				}
			}
			angle = atan(_slope);
			radius = sqrt(pow((xmax-xmin),2) + pow((ymax-ymin),2))/2;

			R = temp;
			ROS_DEBUG("Found line: y = %fx + %f , R val: %f , angle: %f, radius: %f", _slope,_yInt,R,angle,radius);
		}
		return true;
	}
};

class cloud_parse
{
public:
	cloud_parse()
	{
  	// Create a ROS subscriber for the input point cloud
		sub = nh.subscribe ("/cloud_data", 100, &cloud_parse::cloud_cb, this);

  // Create a ROS publisher for the extracted lines
#ifdef RVIZ
		marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
#else
		marker_pub = nh.advertise<slam_node::lm_array>("/landmarks",10);
#endif
		ROS_DEBUG("Initialized cloud_parse object...");
		maxLines = 0;
	}

  // Callback function upon recieving point cloud
	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		ROS_DEBUG("Running cb for recieved point cloud.");
		std::vector<Point> points;
		points.clear();
		std::vector<Line> lines;
		lines.clear();
		Point temp;
  // Container for original & filtered data
		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
		pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
		pcl::PCLPointCloud2 cloud_filtered;
		ros::Rate r(100);

  // Convert to PCL data type
		pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Reduce point cloud size and remove redundant points
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		sor.setInputCloud (cloudPtr);
		sor.setLeafSize (0.1, 0.1, 0.1);
		sor.filter (cloud_filtered);

  // Convert to ROS data type
		sensor_msgs::PointCloud2 output;
		pcl_conversions::fromPCL(cloud_filtered, output);

	// Extract points from pointcloud 
		pcl::PointCloud<pcl::PointXYZ> msg_;
		pcl::fromROSMsg(output,msg_);

	// Set ground reference point
		float ground = 0.18;

	// Filter points by z-coord and add to points vector
		int nPoints = msg_.points.size();
		for(int i=0; i<nPoints; i++) 
		{
			if(msg_.points[i].z >= ground && msg_.points[i].z <= ground + 0.1)
			{ 
				temp._x = msg_.points[i].x;
				temp._y = msg_.points[i].y;
				points.push_back(temp);
				// ROS_DEBUG("Adding point (%f,%f) to set S. ",temp._x,temp._y);
				// r.sleep();
			}
		}

  	// Split and merge line extraction
		lines = split_and_merge(points);

	// Publish and display lines
#ifdef RVIZ
		visualization_msgs::MarkerArray line_strip_array;
		visualization_msgs::Marker line_strip;
		line_strip.header.frame_id = "/world";
		line_strip.ns = "lines";
		line_strip.action = visualization_msgs::Marker::ADD;
		line_strip.pose.orientation.w = 1.0;
		line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_strip markers use only the x component of scale, for the line width
		line_strip.scale.x = 0.1;

    // Line strip is blue
		line_strip.color.b = 1.0;
		line_strip.color.a = 1.0;

    // Create the vertices for the points and lines
		int nLines = lines.size();
		if (nLines > maxLines)
		{
			maxLines = nLines;
		}

		double x,y;
		int nMarkers = 0;
		for (int i=0;i<nLines;i++)
		{
			if (lines[i].nPoints >= 15)
			{
				// Get midpoint of line
				x = (lines[i].xmax + lines[i].xmin) / 2;
				y = (lines[i].ymax + lines[i].ymin) / 2;

				// Create points vector for visualization
				for(double r = -lines[i].radius; r < lines[i].radius;r+=lines[i].radius/20)
				{
					geometry_msgs::Point p;
					p.x = x + r*cos(lines[i].angle);
					p.y = y + r*sin(lines[i].angle);
					p.z = ground;
 
					line_strip.points.push_back(p);
				}
				line_strip.id = nMarkers;
				nMarkers++;
				line_strip.header.stamp = ros::Time::now();
				line_strip_array.markers.push_back(line_strip);
				line_strip.points.clear();
			}
		}

		ROS_DEBUG("Publishing %lu lines.", line_strip_array.markers.size());

		for (int i=nMarkers;i<maxLines;i++)
		{
			line_strip.id = i;
			line_strip.header.stamp = ros::Time::now();
			line_strip_array.markers.push_back(line_strip);
		}

		
		marker_pub.publish(line_strip_array);
		line_strip_array.markers.clear();
#else 
		slam_node::landmark landmark;
		slam_node::lm_array landmarks;
		int nLines = lines.size();
		double x,y;
		for (int i=0;i<nLines;i++)
		{
			x = (lines[i].xmax + lines[i].xmin) / 2;
		    y = (lines[i].ymax + lines[i].ymin) / 2;
			landmark.x = x;
			landmark.y = y;
			landmark.radius = lines[i].radius; 
			landmark.angle = lines[i].angle;
			landmarks.landmarks.push_back(landmark);
		}
		marker_pub.publish(landmarks);
		landmarks.landmarks.clear();
#endif 
	}
	

	// Function runs split and merge algorithm on set of points to find all lines within that set
	std::vector<Line> split_and_merge(std::vector<Point> S)
	{
		ROS_DEBUG("Running split and merge on set S:");
		std::vector<Point> temp_p1, temp_p2, temp_p3;
		std::vector< std::vector<Point> > points;
		std::vector<Line> lines;
		Line line;
		int nSets, nPts;
		double split_threshold = 1e-3, merge_threshold = 1e-2;
		bool split_and_extract = true, merged;
		points.push_back(S);
		ros::Rate r(100);

		while (split_and_extract)
		{
			lines.clear();
			nSets = points.size();
		// For every set try to extract line and check threshold
			for (int i=0;i<nSets;i++)
			{
			//fit line to points
				if (line.fitPoints(points[i]) != true)
				{
					// Remove singular set and do not fit line to it
					ROS_DEBUG("Attempted to fit line to single point.");
					points.erase(points.begin() + i);
				}
				else
				{
					lines.push_back(line);
				}
			}

			nSets = lines.size();
		// For every line check threshold
			bool split = false;
			for (int i=0;i<nSets;i++)
			{
			// if greatest outlier is bigger than threshold distance from line, split set 
				if (lines[i].R > split_threshold)
				{
					ROS_DEBUG("Splitting set S[%i] of size %lu with xmin,xmax (%f,%f) and ymin, ymax(%f,%f)",i, points[i].size(), lines[i].xmin,lines[i].xmax,lines[i].ymin,lines[i].ymax);
					split = true;
					nPts = points[i].size();


					temp_p1.clear();
					temp_p2.clear();
					for (int j=0;j<nPts;j++)
					{
						if (j <= floor(nPts/2))
						{
							ROS_DEBUG("Adding point (%f,%f) to temp set S1",points[i][j]._x, points[i][j]._y);
							temp_p1.push_back(points[i][j]);
						}
						if (j >= floor(nPts/2))
						{
							ROS_DEBUG("Adding point (%f,%f) to temp set S2",points[i][j]._x, points[i][j]._y);
							temp_p2.push_back(points[i][j]);
						}
						// r.sleep();
					}

				// After splitting into two sets, re-run split_and_extract on all sets of points
					points.erase(points.begin() + i);

				// Only add sets which are not singular in size
					if (temp_p1.size() > 1)
					{
						points.push_back(temp_p1);
						ROS_DEBUG("Using temp set S1 with size %lu",temp_p1.size());
					} 

					if (temp_p2.size() > 1)
					{
						points.push_back(temp_p2);
						ROS_DEBUG("Using temp set S2 with size %lu",temp_p2.size());
					} 

					
					temp_p1.clear();
					temp_p2.clear();
					break;
				}
			}

		// If we had a split, re-run loop and clear set of lines
			if (split == false)
			{
				split_and_extract = false;
			}
		}

	// Attempt merging co-linear lines 
		for (int i=0;i<(lines.size()-1);i++)
		{
			merged = false;
		// check if lines co-linear with rest of lines in set 
			for (int j=i+1;j<lines.size();j++)
			{
				if(merged)
				{
					i--;
					break;
				}
			// if co-linear check fit, otherwise change nothing
				if(std::fabs(std::fabs(lines[i].angle) - std::fabs(lines[j].angle)) <= 0.2 ) 
				{
					ROS_DEBUG("Checking if co-linear lines can be merged");
					temp_p3.clear();
				// First add all points into temp set 3
					for(int k=0;k<points[i].size();k++)
					{
						temp_p3.push_back(points[i][k]);
					}
					for(int k=0;k<points[j].size();k++)
					{
						temp_p3.push_back(points[j][k]);
					}

				// check R value of line fit to see if we are merging
					if (line.fitPoints(temp_p3) == true)
					{
					// if line works remove old lines and sets of points, add new line and set
						if(line.R < merge_threshold)
						{
							ROS_DEBUG("From set of %lu lines, merging lines %i and %i.",lines.size(),i,j);
							lines.erase(lines.begin() + i);
							points.erase(points.begin() + i);
							lines.erase(lines.begin() + (j-1));
							points.erase(points.begin() + (j-1));
							points.push_back(temp_p3);
							lines.push_back(line);
							merged = true;
						}
					}
				}
			}

		}
		nSets = lines.size();
		ROS_DEBUG("Found %i lines to fit PointCloud. ",nSets);

		// Print some debug info on final set of lines saved
		for (int i=0;i<nSets;i++)
		{
			ROS_DEBUG("Line %i size: %lu", i,lines[i].nPoints);
		}
		return lines;
	}

private:
	ros::Publisher marker_pub;
	ros::Subscriber sub;
	ros::NodeHandle nh; 
	int maxLines;

};//End of class cloud_cb_pub

int main (int argc, char** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "spm_node");
	cloud_parse cloud_parse_obj;

	// Spinning handles ROS events
	ros::spin();
	
	return 0;
}