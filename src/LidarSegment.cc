#include "LidarSegment.h"
#include <iostream>

// Default Constructor
LidarSegment::LidarSegment(void) {
	distance_threshold_ = 0.05;
	radius_threshold_ = 0.5;
}  // LidarSegment

// Custom constructor
LidarSegment::LidarSegment(const std::vector<Eigen::Vector2f>& point_list,
													 const std::vector<float>& range_data) {
	distance_threshold_ = 0.05;
	radius_threshold_ = 0.5;
	full_points_.resize(point_list.size());
	range_data_.resize(point_list.size());
	for (int point_index = 0; point_index < point_list.size(); ++point_index) {
		full_points_[point_index] = point_list[point_index];
		range_data_[point_index] = range_data[point_index];
	}
}  // LidarSegment

// Custom constructor
LidarSegment::LidarSegment(const std::vector<Eigen::Vector2f>& point_list,
													 const std::vector<float>& range_data,
													 float thresh) {
	distance_threshold_ = thresh;
	radius_threshold_ = 0.5;
	full_points_.resize(point_list.size());
	range_data_.resize(point_list.size());
	for (int point_index = 0; point_index < point_list.size(); ++point_index) {
		full_points_[point_index] = point_list[point_index];
		range_data_[point_index] = range_data[point_index];
	}
}  // LidarSegment

// Calculate then return the list of segmented points
std::vector<Eigen::Vector2f> LidarSegment::segmented_points(void) {
	// Pre-process points to do clustering
	if (full_points_.size() > 0) {
		std::vector<int> low_segment(full_points_.size());
		for (int index = 0; index < full_points_.size(); ++index) {
			low_segment[index] = index;
		}
		std::vector<int> high_segment(2);
		high_segment[0] = full_points_.size()-1;
		high_segment[1] = 0;
		std::vector< std::vector<int> > segments(2);
		segments[0] = low_segment;
		segments[1] = high_segment;
		segmented_indices_.clear();

		// Cluster based on radii difference
		PerformClustering(segments);
		// Segment using split and merge
		int start_index = 0;
		PerformSplitAndMerge(segments, start_index);
		/*
		std::cout << "Segments: ";
		for (int i = 0; i < segments.size(); ++i) {
			std::cout << "(";
			for (int j = 0; j < segments[i].size(); ++j) {
				std::cout << segments[i][j];
				if (j < segments[i].size() - 1) {
					std::cout << ", ";
				} else {
					std::cout << ")";
				}
			}
			if (i < segments.size() -1) {
				std::cout << ", ";
			} else {
				std::cout << std::endl;
			}
		}
		int k;
		std::cin >> k;

		std::cout << "Adjust end" << std::endl;
		*/
		segments.back().insert(segments.back().begin(),
													 segments[segments.size() - 2].front());
		segments.back().push_back(segments.front().back());
		segments.erase(segments.begin() + segments.size() - 2);
		segments.erase(segments.begin());
		/*
		for (int i = 0; i < segments.size(); ++i) {
			std::cout << "(";
			for (int j = 0; j < segments[i].size(); ++j) {
				std::cout << segments[i][j];
				if (j < segments[i].size() - 1) {
					std::cout << ", ";
				} else {
					std::cout << ")";
				}
			}
			if (i < segments.size() -1) {
				std::cout << ", ";
			} else {
				std::cout << std::endl;
			}
		}
		std::cin >> k;
		std::cout << "Re-segment" << std::endl;
		*/
		start_index = 0;
		segmented_indices_.clear();
		PerformSplitAndMerge(segments, start_index);
		segmented_indices_.push_back(segmented_indices_[0]);
		/*
		for (int i = 0; i < segments.size(); ++i) {
			std::cout << "(";
			for (int j = 0; j < segments[i].size(); ++j) {
				std::cout << segments[i][j];
				if (j < segments[i].size() - 1) {
					std::cout << ", ";
				} else {
					std::cout << ")";
				}
			}
			if (i < segments.size() -1) {
				std::cout << ", ";
			} else {
				std::cout << std::endl;
			}
		}
		std::cout << std::endl << "Segmented indices: ";
		for (int i = 0; i < segmented_indices_.size(); ++i) {
			std::cout << segmented_indices_[i];
			if (i < segmented_indices_.size() - 1) {
				std::cout << ",";
			} else {
				std::cout << std::endl;
			}
		}
		std::cin >> k;
		*/
		
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
void LidarSegment::PerformClustering(std::vector< std::vector<int> >& segments) {
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
}  // PerformClustering

// Perform computation of segmentation into lines
void LidarSegment::PerformSplitAndMerge(std::vector<std::vector<int> >& segments,
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

// Split a line segment at the furthest point
std::vector< std::vector<int> >
LidarSegment::SplitAtFurthestPoint(const std::vector<int>& segment) {
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
LidarSegment::SplitSegment(const std::vector<int>& segment, int split_index) {
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
							 
LidarSegment::Point
LidarSegment::PointFarthestFromLine(const std::vector<int>& point_list) {
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
float LidarSegment::FindDistance(const Eigen::Vector2f& a,
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


