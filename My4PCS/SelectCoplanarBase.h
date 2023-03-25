#pragma once
#include <string>
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
using namespace std;
class SelectCoplanarBase
{

public:
	pcl::PointCloud<pcl::PointXYZ> cloud;
	inline void LoadPointCloud(string filepath);
	pcl::PointCloud<pcl::PointXYZ> getBase(string filepath);

};

