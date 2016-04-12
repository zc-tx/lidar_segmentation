#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "LidarSegment2d.h"

#include <vector>
#include <Eigen/Dense>

typedef Eigen::Vector2f Vec2;

bool new_data = true;
sensor_msgs::LaserScan laser_in;
void laser_callback(const sensor_msgs::LaserScan& laser_input) {
  laser_in = laser_input;
  new_data = true;
}  // laser_callback

// Show the final desired trajectory as a green line in rviz 
void SetupFinalTrajectoryVisualization(visualization_msgs::Marker& trajectory,
																			 int id) {
	trajectory.header.frame_id = "world";
	trajectory.id = id;
	trajectory.type = visualization_msgs::Marker::LINE_STRIP;
	trajectory.action = visualization_msgs::Marker::ADD;
	trajectory.scale.x = 0.04;
	trajectory.color.a = 1.0;
	trajectory.color.g = 1.0;
}

// Show every lidar point as a red dot in rviz
void SetupFullLaserVisualization(visualization_msgs::Marker& full_laser,
																 int id) {
	full_laser.header.frame_id = "world";
	full_laser.id = id;
	full_laser.type = visualization_msgs::Marker::POINTS;
	full_laser.action = visualization_msgs::Marker::ADD;
	full_laser.scale.x = 0.04;
	full_laser.scale.y = 0.04;
	full_laser.color.a = 1.0;
	full_laser.color.r = 1.0;
}

// Show only the segmented vertices as white dots in rviz
void SetupSegmentedLaserVisualization(visualization_msgs::Marker& seg_laser,
																			int id) {
	seg_laser.header.frame_id = "world";
	seg_laser.id = id;
	seg_laser.type = visualization_msgs::Marker::POINTS;
	seg_laser.action = visualization_msgs::Marker::ADD;
	seg_laser.scale.x = 0.05;
	seg_laser.scale.y = 0.05;
	seg_laser.color.a = 1.0;
	seg_laser.color.r = 1.0;
	seg_laser.color.g = 1.0;
	seg_laser.color.b = 1.0;
}

// Show black lines between the segmented vertices, representing the walls the
// quadrotor sees
void SetupSegmentedLinesVisualization(visualization_msgs::Marker& laser_lines,
																			int id) {
	laser_lines.header.frame_id = "world";
	laser_lines.id = id;
	laser_lines.type = visualization_msgs::Marker::LINE_STRIP;
	laser_lines.action = visualization_msgs::Marker::ADD;
	laser_lines.scale.x = 0.035;
	laser_lines.color.a = 1.0;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "lidar_segmentation_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);

	ros::Subscriber laser_scan = nh.subscribe("scan", 1, laser_callback);
  ros::Publisher full_point_pub =
		nh.advertise<visualization_msgs::Marker>("laser_full_points_vis", 0);
	ros::Publisher seg_point_pub =
		nh.advertise<visualization_msgs::Marker>("laser_seg_points_vis", 0);
	ros::Publisher line_pub =
		nh.advertise<visualization_msgs::Marker>("laser_lines_vis",0);
	
	double distance_threshold;
	if (nh.getParam("/dist_thresh", distance_threshold)) {;}
	else {
		//ROS_ERROR("Set Distance Threshold");
		//return 0;
		distance_threshold = 0.1;
	}
	
	visualization_msgs::Marker full_laser_points;
	SetupFullLaserVisualization(full_laser_points, 3);
	visualization_msgs::Marker seg_laser_points;
	SetupSegmentedLaserVisualization(seg_laser_points, 4);
	visualization_msgs::Marker laser_lines;
	SetupSegmentedLinesVisualization(laser_lines, 5);

	while(ros::ok()) {
		ros::spinOnce();

		if (new_data) {
		  new_data = false;
		  std::vector<Eigen::Vector2f> full_point_list(laser_in.ranges.size());
		  std::vector<float> range_list(laser_in.ranges.size());
		  full_laser_points.points.clear();
		  
		  float theta_rel = -M_PI/4.0 + laser_in.angle_min;
		  Eigen::Vector2f tmp_point;
			for (int data_index = 0; data_index < laser_in.ranges.size();
					 ++data_index) {
				range_list[data_index] = laser_in.ranges[data_index];
				if (range_list[data_index] > 6.0) {
				  range_list[data_index] = 6.0;
				}
				tmp_point[0] =  range_list[data_index]*cos(theta_rel);
				tmp_point[1] = -range_list[data_index]*sin(theta_rel);
				full_point_list[data_index] = tmp_point;

				geometry_msgs::Point laser_point;
				laser_point.x = tmp_point[0];
				laser_point.y = tmp_point[1];
				laser_point.z = 0.02;
				full_laser_points.points.push_back(laser_point);

				theta_rel += laser_in.angle_increment;
			}
			
      LidarSegment2d lidar_full_points(full_point_list,
																			 range_list,
																			 distance_threshold);
			std::vector<Eigen::Vector2f> lidar_segmented_points = lidar_full_points.segmented_points();

			// Store the segmented points for rviz visualization
			seg_laser_points.points.clear();
			laser_lines.points.clear();
			geometry_msgs::Point rviz_point;
			for (int index = 0; index < lidar_segmented_points.size(); ++index) {
				rviz_point.x = lidar_segmented_points[index][0];
				rviz_point.y = lidar_segmented_points[index][1];
				rviz_point.z = 0.05;
				seg_laser_points.points.push_back(rviz_point);
				rviz_point.z = 0.0;
				laser_lines.points.push_back(rviz_point);
			}
			
			full_laser_points.header.stamp = ros::Time();
			seg_laser_points.header.stamp = ros::Time();
			laser_lines.header.stamp = ros::Time();
		
			full_point_pub.publish(full_laser_points);
			seg_point_pub.publish(seg_laser_points);
			line_pub.publish(laser_lines);

			new_data = false;
		}
		loop_rate.sleep();
	}
	return 0;
}
