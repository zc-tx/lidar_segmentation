#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "LidarSegment.h"

#include <vector>
#include <Eigen/Dense>

typedef Eigen::Vector2f Vec2;

visualization_msgs::Marker full_laser_points;
std::vector<Vec2> full_point_list;
std::vector<float> range_data;
bool new_data = true;
void laser_callback(const sensor_msgs::LaserScan& laser_in) {
	full_laser_points.points.clear();
	full_laser_points.points.resize(laser_in.ranges.size());

	full_point_list.clear();
	full_point_list.resize(laser_in.ranges.size());
	
	range_data.clear();
	range_data.resize(laser_in.ranges.size());
	float theta = laser_in.angle_min;
	Vec2 tmp_point;
	for (int data_index = 0; data_index < laser_in.ranges.size(); ++data_index) {
		geometry_msgs::Point laser_point;
		laser_point.x = laser_in.ranges[data_index]*cos(theta);
		laser_point.y = -laser_in.ranges[data_index]*sin(theta);
		laser_point.z = 0.02;
		full_laser_points.points[data_index] = laser_point;
		theta += laser_in.angle_increment;

		tmp_point[0] = laser_point.x;
		tmp_point[1] = laser_point.y;
		full_point_list[data_index] = tmp_point;
		range_data[data_index] = laser_in.ranges[data_index];
	}
	new_data = true;
}
int main(int argc, char* argv[]) {
	ros::init(argc, argv, "lidar_segmentation_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);

	ros::Subscriber laser_scan = nh.subscribe("laser_scan", 1, laser_callback);
	ros::Publisher full_point_pub = nh.advertise<visualization_msgs::Marker>("laser_full_points_vis", 0);
	ros::Publisher seg_point_pub = nh.advertise<visualization_msgs::Marker>("laser_seg_points_vis", 0);
	ros::Publisher line_pub = nh.advertise<visualization_msgs::Marker>("laser_lines_vis",0);
	ros::Publisher quad_pub = nh.advertise<visualization_msgs::Marker>("quad_dummy_vis", 0);
	
	visualization_msgs::Marker quad_dummy;
	quad_dummy.header.frame_id = "world";
	quad_dummy.id = 0;
	quad_dummy.type = visualization_msgs::Marker::SPHERE;
	quad_dummy.action = visualization_msgs::Marker::ADD;
	quad_dummy.scale.x = 0.282;
	quad_dummy.scale.y = 0.282;
	quad_dummy.scale.z = 0.282;
	quad_dummy.color.a = 1.0;
	quad_dummy.color.b = 1.0;

	full_laser_points.header.frame_id = "world";
	full_laser_points.id = 1;
	full_laser_points.type = visualization_msgs::Marker::POINTS;
	full_laser_points.action = visualization_msgs::Marker::ADD;
	full_laser_points.scale.x = 0.04;
	full_laser_points.scale.y = 0.04;
	full_laser_points.color.a = 1.0;
	full_laser_points.color.r = 1.0;

	visualization_msgs::Marker seg_laser_points;
	seg_laser_points.header.frame_id = "world";
	seg_laser_points.id = 2;
	seg_laser_points.type = visualization_msgs::Marker::POINTS;
	seg_laser_points.action = visualization_msgs::Marker::ADD;
	seg_laser_points.scale.x = 0.05;
	seg_laser_points.scale.y = 0.05;
	seg_laser_points.color.a = 1.0;
	seg_laser_points.color.r = 1.0;
	seg_laser_points.color.g = 1.0;
	seg_laser_points.color.b = 1.0;
	
	visualization_msgs::Marker laser_lines;
	laser_lines.header.frame_id = "world";
	laser_lines.id = 3;
	laser_lines.type = visualization_msgs::Marker::LINE_STRIP;
	laser_lines.action = visualization_msgs::Marker::ADD;
	laser_lines.scale.x = 0.035;
	laser_lines.color.a = 1.0;

	double distance_threshold;
	if (nh.getParam("/dist_thresh", distance_threshold)) {;}
	else {
		ROS_ERROR("Set Distance Threshold");
		return 0;
	}
	
	while(ros::ok()) {
		ros::spinOnce();

		if (new_data) {
			LidarSegment points(full_point_list, range_data, distance_threshold);
			std::vector<Eigen::Vector2f> segmented_point_list = points.segmented_points();

			geometry_msgs::Point line_point;	
			seg_laser_points.points.clear();
			laser_lines.points.clear();
			for (int line_index = 0; line_index < segmented_point_list.size(); ++line_index) {
				line_point.x = segmented_point_list[line_index][0];
				line_point.y = segmented_point_list[line_index][1];
				line_point.z = 0.05;
				seg_laser_points.points.push_back(line_point);
				line_point.z = 0.0;
				laser_lines.points.push_back(line_point);
			}
			full_laser_points.header.stamp = ros::Time();
			seg_laser_points.header.stamp = ros::Time();
			laser_lines.header.stamp = ros::Time();
		
			quad_pub.publish(quad_dummy);
			full_point_pub.publish(full_laser_points);
			seg_point_pub.publish(seg_laser_points);
			line_pub.publish(laser_lines);

			new_data = false;
		}
		loop_rate.sleep();
	}
	return 0;
}
