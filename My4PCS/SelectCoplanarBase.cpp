#include "SelectCoplanarBase.h"
using namespace std;
inline void SelectCoplanarBase::LoadPointCloud(string filepath) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_ptr = this->cloud;
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filepath, *cloud_ptr) == -1) {
		std::cerr << "can't load point cloud file" << std::endl;
	}
}
pcl::PointCloud<pcl::PointXYZ> SelectCoplanarBase::getBase(string filepath) {
	pcl::PointCloud<pcl::PointXYZ> base;
	LoadPointCloud(filepath);
	if (cloud.size() == 0) {
		std::cerr << "Load Fail" << std::endl;
	}
	cout << "File Loaded " << cloud.size() << " points" << endl;

	return base;
}