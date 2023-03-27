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
	void SelectCoplanarBase();//ѡ��4����Ϊbase
	void FindCongruent(double);//�ҵ�Ŀ������з��ϸ��Ա任�Ļ�
	vector<Eigen::Vector3d> VectorDecomp(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z);//��z�ֽ⵽��ֱ��xyƽ���xy�ϵ�ͶӰ
	double Point2TriangleDist(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z, Eigen::Vector3d w);//�㵽�����εľ���
	double Line2LineDist(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1);
	bool CheckLegal(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z);
};

