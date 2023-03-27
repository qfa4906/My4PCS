#pragma once
#include <vector>
#include <tuple>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
using namespace std;
class FourPCS
{
public:
	FourPCS(pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>);
//private:
	pcl::PointCloud<pcl::PointXYZ> P_cloud;
	pcl::PointCloud<pcl::PointXYZ> Q_cloud;
	pcl::PointCloud<pcl::PointXYZ> U_cloud;
	pcl::PointCloud<pcl::PointXYZ> Base;
	string filepath; 
	void SelectCoplanarBase();//选择4点作为base
	void FindCongruent(double);//找到目标点云中符合刚性变换的基
	vector<Eigen::Vector3d> VectorDecomp(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z);//将z分解到垂直于xy平面和xy上的投影
	double Point2TriangleDist(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z, Eigen::Vector3d w);//点到三角形的距离
	double Line2LineDist(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1);
	bool CheckLegal(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z);
};

