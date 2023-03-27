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
	pcl::PointCloud<pcl::PointXYZ> FindMatchPairs(pcl::PointCloud<pcl::PointXYZ>& ,double d, double delta);//寻找目标点云的对应点对
	double Line2LineDist(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1);//直线距离
	void Line2LineLeastSquareIntersection(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1, double* r1, double* r2);//直线最小二乘交点交比
	bool CheckLegal(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z);//检测第四点位置
	pcl::PointXYZ NormalizePointXYZ(pcl::PointXYZ);//归一化pcl点
	friend pcl::PointXYZ operator*(double k, pcl::PointXYZ p);
	friend pcl::PointXYZ operator+(pcl::PointXYZ p1, pcl::PointXYZ p2);
	friend pcl::PointXYZ operator-(pcl::PointXYZ p1, pcl::PointXYZ p2);
};

