#ifndef LIDAR_SEGMENT_2D_H_
#define LIDAR_SEGMENT_2D_H_

#include <vector>
#include <Eigen/Dense>

class LidarSegment2d {
public:
	LidarSegment2d(void);
	LidarSegment2d(const std::vector<Eigen::Vector2f>& point_list);
	LidarSegment2d(const std::vector<Eigen::Vector2f>& point_list,
								 float threshold);
	LidarSegment2d(const std::vector<Eigen::Vector2f>& point_list,
							 const std::vector<float>& range_data);
	LidarSegment2d(const std::vector<Eigen::Vector2f>& point_list,
							 const std::vector<float>& range_data,
							 float threshold);	

	std::vector<Eigen::Vector2f> segmented_points(void);
	
private:
	struct Point {
	public:
		int index_;
		float distance_;
	};

	std::vector<float> range_data_;
	std::vector<Eigen::Vector2f> full_points_;
	std::vector<int> segmented_indices_;
	std::vector<Eigen::Vector2f> segmented_points_;
	float radius_threshold_;  // For clustering
	float distance_threshold_;  // For split and merge clustering

	void PerformClustering(std::vector< std::vector<int> >& segments);
	void PerformSplitAndMerge(std::vector< std::vector<int> >& segments, int& index);
	std::vector< std::vector<int> > SplitAtFurthestPoint(const std::vector<int>& segment);
	std::vector< std::vector<int> > SplitSegment(const std::vector<int>& segment, int index);
	Point PointFarthestFromLine(const std::vector<int>& point_list);
	float FindDistance(const Eigen::Vector2f& a,
										 const Eigen::Vector2f& b,
										 const Eigen::Vector2f& c);
};

#endif
