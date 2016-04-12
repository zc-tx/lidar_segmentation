#include "LidarSegment2d.h"
#include <iostream>

// Default Constructor
LidarSegment2d::LidarSegment2d(void) {
	distance_threshold_ = 0.05;
	radius_threshold_ = 0.5;
}  // LidarSegment2d

LidarSegment2d::LidarSegment2d(const std::vector<Eigen::Vector2f>& point_list) {
	distance_threshold_ = 0.05;
	radius_threshold_ = 0.5;
	full_points_.resize(point_list.size());
	full_points_ = point_list;
}  // LidarSegment2d

LidarSegment2d::LidarSegment2d(const std::vector<Eigen::Vector2f>& point_list,
															 float thresh) {
	distance_threshold_ = thresh;
	radius_threshold_ = 0.5;
	full_points_.resize(point_list.size());
	full_points_ = point_list;
}  // LidarSegment2d

// Custom constructor
LidarSegment2d::LidarSegment2d(const std::vector<Eigen::Vector2f>& point_list,
													 const std::vector<float>& range_data) {
	distance_threshold_ = 0.05;
	radius_threshold_ = 0.5;
	full_points_.resize(point_list.size());
	full_points_ = point_list;
	range_data_.resize(range_data.size());
	range_data_ = range_data;
}  // LidarSegment2d

// Custom constructor
LidarSegment2d::LidarSegment2d(const std::vector<Eigen::Vector2f>& point_list,
													 const std::vector<float>& range_data,
													 float thresh) {
	distance_threshold_ = thresh;
	radius_threshold_ = 0.5;
	full_points_.resize(point_list.size());
	full_points_ = point_list;
	range_data_.resize(range_data.size());
	range_data_ = range_data;
}  // LidarSegment2d

// Calculate then return the list of segmented points
std::vector<Eigen::Vector2f> LidarSegment2d::segmented_points(void) {
	// Pre-process points to do clustering
	if (full_points_.size() > 0) {
		std::vector<int> low_segment(full_points_.size());
		for (int index = 0; index < low_segment.size(); ++index) {
			low_segment[index] = index;
		}
		// Set up segments so the points are in two groups as
		// (v_0, v_1, v_2, ... v_n-1, v_n), (v_n, v_0)
		std::vector<int> high_segment(2);
		high_segment[0] = full_points_.size()-1;
		high_segment[1] = 0;
		std::vector< std::vector<int> > segments(2);
		segments[0] = low_segment;
		segments[1] = high_segment;
		segmented_indices_.clear();

		// Cluster based on radii difference
		// If the radii of two points in a group are different by more than
		// radius_threshold_ than the group will be split, i.e.
		// (v_0, v_1, v_2, v_3) --> (v_0, v_1), (v_1, v_2, v_3)
		PerformClustering(segments);
		
		// Segment using split and merge
		// Takes the pre-clustered groups and performs split-and-merge
		// The algorithm is recursive unti groups of two remain.
		// A line is made between the start and end of each cluster and the distance
		// of every other point in the cluster to that line is calculated. If that
		// distance is greater than distance_threshold_ than the cluster is split
		// into two clusters at that point. If the points are within the value of
		// distane_threshold_, they are removed from the group leaving only the ends
		int start_index = 0;
		PerformSplitAndMerge(segments, start_index);

		// The initial split-and-merge leaves the initial and final vertex as a line
		// which isn't always the true segmentation,
		// i.e. (v_0, v_a), (v_a, v_b) (v_b, v_n), (v_n, v_0)
		// This adjusts the final segment to be (v_b, v_n, v_0, v_a) and removes the
		// first segment. The split and merge is then recalculated to eliminate
		// v_0 and v_n if they're not true segmented vertices
		segments.back().insert(segments.back().begin(),
													 segments[segments.size() - 2].front());
		segments.back().push_back(segments.front().back());
		segments.erase(segments.begin() + segments.size() - 2);
		segments.erase(segments.begin());

		start_index = 0;
		segmented_indices_.clear();
		PerformSplitAndMerge(segments, start_index);
		segmented_indices_.push_back(segmented_indices_[0]);
		
		segmented_points_.resize(segments.size() + 1);
		int i = 0;
		for (; i < segments.size(); ++i) {
			segmented_points_[i] = full_points_[segmented_indices_[i]];
		}
		segmented_points_[i] = full_points_[segmented_indices_[0]];
	}
	return segmented_points_;
} // segmented_points

// Perform an initial clustering from radius criterion
void LidarSegment2d::PerformClustering(std::vector< std::vector<int> >& segments) {
	if (range_data_.size() > 0) {
		for (int segment_index = 0; segment_index < segments.size(); ++segment_index) {
			if (segments[segment_index].size() > 2) {  // skip groups of 2
				for (int point_index = 1; point_index < segments[segment_index].size();
						 ++point_index) {
					if(abs(range_data_[segments[segment_index][point_index]] -
								 range_data_[segments[segment_index][point_index - 1]]) >
						 radius_threshold_) {  // radial distance between two is too big, split
						std::vector< std::vector<int> > split_segments =
							SplitSegment(segments[segment_index], point_index);
						segments[segment_index].swap(split_segments[1]);
						segments.insert(segments.begin() + segment_index, split_segments[0]);
					}
				}
			}
		}
	}
}  // PerformClustering

// Perform computation of segmentation into lines
void LidarSegment2d::PerformSplitAndMerge(std::vector<std::vector<int> >& segments,
																				int& segment_index) {
 	for (; segment_index < segments.size(); ++segment_index) {
		if (segments[segment_index].size() > 2) {  // line is already finished
			std::vector< std::vector<int> > split_segments =
				SplitAtFurthestPoint(segments[segment_index]);
			if(split_segments.size() > 1) {  // line was split
				segments[segment_index].swap(split_segments[1]);
				segments.insert(segments.begin() + segment_index, split_segments[0]);
			} else {  // line was not split form distance criterion
				segments[segment_index].swap(split_segments[0]);
			}
			PerformSplitAndMerge(segments, segment_index);
		} else {
			segmented_indices_.push_back(segments[segment_index][0]);
		}
	}
}  // PerformSegmentation

// Split a line segment at the point in the cluster furthest from the line
std::vector< std::vector<int> >
LidarSegment2d::SplitAtFurthestPoint(const std::vector<int>& segment) {
	Point break_point = PointFarthestFromLine(segment);
	std::vector< std::vector<int> >segments(2);
	if (break_point.distance_ > distance_threshold_) {
		segments = SplitSegment(segment, break_point.index_);
	} else {
		std::vector<int> full_segment(2);
		full_segment[0] = segment.front();
		full_segment[1] = segment.back();
		segments.resize(1);
		segments[0] = full_segment;
	}
	return segments;
}  // SplitLineSegment

// Split a line segment at a given index
std::vector< std::vector<int> >
LidarSegment2d::SplitSegment(const std::vector<int>& segment, int split_index) {
	std::vector<int> low_segment(split_index + 1);
	for (int low_index = 0; low_index < split_index + 1; ++low_index) {
		low_segment[low_index] = segment[low_index];
	}
	std::vector<int> high_segment(segment.size() - split_index);
	for (int high_index = split_index; high_index < segment.size(); ++high_index) {
		high_segment[high_index-split_index] = segment[high_index];
	}
	std::vector< std::vector<int> > segments(2);
	segments[0] = low_segment;
	segments[1] = high_segment;
	return segments;
}  // SplitSegment
							 
LidarSegment2d::Point
LidarSegment2d::PointFarthestFromLine(const std::vector<int>& point_list) {
	float max_distance = FindDistance(full_points_[point_list.front()],
																		full_points_[point_list.back()],
																		full_points_[point_list[1]]);
	int max_index = 1;
	for (int point_index = 2; point_index < point_list.size()-1; ++point_index) {
		float distance = FindDistance(full_points_[point_list.front()],
																	full_points_[point_list.back()],
																	full_points_[point_list[point_index]]);
		if (distance > max_distance) {
			max_distance = distance;
			max_index = point_index;
		}
	}
	Point farthest_point;
	farthest_point.index_ = max_index;
	farthest_point.distance_ = max_distance;
	return farthest_point;
}  // PointFarthestFromLine

// Find a the distance between point C and its projection onto the segment
// between points a and b
float LidarSegment2d::FindDistance(const Eigen::Vector2f& a,
																 const Eigen::Vector2f& b,
																 const Eigen::Vector2f& c) {
	Eigen::Vector2f ba_hat = (b-a).normalized();
	float t = (c-a).transpose()*ba_hat;
	if (t < 0 ) {  // outside a
		return (c-a).norm();
	} else if (t > (b-a).norm()) {  // outside b
		return (c-b).norm();
	}
	return (c-a-t*ba_hat).norm();
}  // FindDistance


