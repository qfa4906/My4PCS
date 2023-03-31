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
	pcl::PointCloud<pcl::PointXYZ> congruent_base;
	pcl::PointCloud<pcl::PointXYZ> save_cloud, save_cloud2;
	string filepath; 
	void SelectCoplanarBase();//ѡ��4����Ϊbase
	void FindCongruent(double);//�ҵ�Ŀ������з��ϸ��Ա任�Ļ�
	vector<Eigen::Vector3d> VectorDecomp(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z);//��z�ֽ⵽��ֱ��xyƽ���xy�ϵ�ͶӰ
	double Point2TriangleDist(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z, Eigen::Vector3d w);//�㵽�����εľ���
	void FindMatchPairs(pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>& , double delta1, double delta2, double d);//Ѱ��Ŀ����ƵĶ�Ӧ���
	double Line2LineDist(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1);//ֱ�߾���
	void Line2LineLeastSquareIntersection(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1, double* r1, double* r2);//ֱ����С���˽��㽻��
	bool CheckLegal(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z);//�����ĵ�λ��
	pcl::PointXYZ NormalizePointXYZ(pcl::PointXYZ);//��һ��pcl��
	Eigen::Matrix4d FindRigidTransformation(pcl::PointXYZ b1, pcl::PointXYZ b2 , pcl::PointXYZ b3, pcl::PointXYZ b4, pcl::PointXYZ p1 , pcl::PointXYZ p2 , pcl::PointXYZ p3 , pcl::PointXYZ p4 );
};

