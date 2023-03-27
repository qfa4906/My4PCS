#include "FourPCS.h"
#include<Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include<math.h>
using namespace std; 
FourPCS::FourPCS(pcl::PointCloud<pcl::PointXYZ> cloud1, pcl::PointCloud<pcl::PointXYZ> cloud2) {
	P_cloud = cloud1;
	Q_cloud = cloud2;
}

void FourPCS::SelectCoplanarBase() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud = P_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sample(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_sample->width = 4;
	cloud_sample->height = 1;
	double MaxArea = 0;
	for (int i = 0; i < 100; i++) {//随机采样100次 选择围成面积最大的三个点
		pcl::RandomSample<pcl::PointXYZ> rs;
		rs.setInputCloud(cloud);
		//设置输出点的数量   
		rs.setSample(4);
		//下采样并输出到cloud_sample
		rs.filter(*cloud_sample);
		Eigen::Vector3d p1(3),p2(3),p3(3);
		p1 << cloud_sample->points[0].x, cloud_sample->points[0].y, cloud_sample->points[0].z;
		p2 << cloud_sample->points[1].x, cloud_sample->points[1].y, cloud_sample->points[1].z;
		p3 << cloud_sample->points[2].x, cloud_sample->points[2].y, cloud_sample->points[2].z;
		Eigen::Vector3d cross = (p1-p2).cross(p2-p3);
		double Area = cross.norm();
		if (Area > MaxArea) {
			MaxArea = Area;
			Base = *cloud_sample;
			cout << "max_area: " << Area << endl;
			cout << "3base: (" << cloud_sample->points[0].x << "," << cloud_sample->points[0].y << "," <<cloud_sample->points[0].z << ")"<<endl;
			cout << "(" << cloud_sample->points[1].x << "," << cloud_sample->points[1].y << "," << cloud_sample->points[1].z << ")" << endl;
			cout << " (" << cloud_sample->points[2].x << "," << cloud_sample->points[2].y << "," << cloud_sample->points[2].z << ")" << endl;
		}
	}
	double norm_norm = 0;
	double max_dist = 0;
	double threh = 0.1;
	Eigen::Vector3d best_point(3);
	for (int j = 0; j < cloud->size();j++) {//遍历点云，选择最优第四点
		//if (j % 100 == 0)
		//	cout << "max_dist: " << max_dist << "norm: " << norm_norm << endl;
		Eigen::Vector3d p1(3), p2(3), p3(3), p4(3);
		p1 << Base.points[0].x, Base.points[0].y, Base.points[0].z;
		p2 << Base.points[1].x, Base.points[1].y, Base.points[1].z;
		p3 << Base.points[2].x, Base.points[2].y, Base.points[2].z;
		p4 << cloud->points[j].x, cloud->points[j].y, cloud->points[j].z;//选取的第四点
		if (p1 == p4 || p2 == p4 || p3 == p4)
		{
			continue;
		}
		if (!CheckLegal(p1-p2, p1-p3, p1-p4) || !CheckLegal(p2 - p1, p2 - p3, p2 - p4) || !CheckLegal(p3 - p1, p3 - p2, p3 - p4))
		{
			continue;
		}
		Eigen::Vector3d norm = VectorDecomp(p1-p2, p1-p3, p1-p4)[0];
		double temp = Point2TriangleDist(p1, p2, p3, p4);
		if (max_dist < temp && norm.norm() < threh) {
			norm_norm = norm.norm();
			max_dist = temp;
			best_point = p4;
			cout << "max_dist: " << max_dist << "norm: " << norm_norm << endl;
		}
	}
	if (max_dist == 0)
		cerr << "未找到符合条件的基，请重试或调整参数" << endl;
	cout << "final norm " << norm_norm << endl;
	cout << "final dist " << max_dist << endl;
	Base.points[3].x = best_point[0];
	Base.points[3].y = best_point[1];
	Base.points[3].z = best_point[2];
}

bool FourPCS::CheckLegal(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {
	Eigen::Matrix3d A = Eigen::Matrix3d::Zero(3, 3);
	vector<Eigen::Vector3d> resault = VectorDecomp(p1, p2, p3);
	Eigen::Vector2d b;
	b << resault[1][0], resault[1][1];
	A.col(0) = p1;
	A.col(1) = p2;
	Eigen::Matrix2d B = A.block<2,2>(0,0);// A.block<2, 2>(0, 0);
	Eigen::Vector2d lamda = B.colPivHouseholderQr().solve(b);
	if (lamda[0] * lamda[1] > 0 && lamda[0] + lamda[1] < 1)
		return false;
	else
		return true;
}


vector<Eigen::Vector3d> FourPCS::VectorDecomp(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z) {
	vector<Eigen::Vector3d> value(2);
	Eigen::Vector3d norm = x.cross(y).normalized();
	Eigen::Vector3d normalized_norm = norm.dot(z) * norm;//求xy平面法线
	Eigen::Vector3d projection = z - normalized_norm;//求z到xy投影
	value[0] = normalized_norm;
	value[1] = projection;
	return value;
}

double FourPCS::Point2TriangleDist(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z, Eigen::Vector3d w) {
	double dist = 0,temp = 0;
	dist = (x - y).cross(x - w).norm()/ (x - y).norm();
	if (((y - z).cross(y - w).norm())/ (y - z).norm() > dist)
		dist = ((y - z).cross(y - w).norm()) / (y - z).norm();
	if (((z - x ).cross(z - w).norm())/ (z - x).norm() > dist)
		dist = ((z - x).cross(z - w).norm()) / (z - x).norm();
	return dist;
}

double FourPCS::Line2LineDist(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1) {
	double a, b, c, d, e;
	Eigen::Vector3d u = u1 - u0;
	Eigen::Vector3d v = v1 - v0;
	Eigen::Vector3d w0 = u0 - v0;
	a = u.dot(u);
	b = u.dot(v);
	c = v.dot(v);
	d = u.dot(w0);
	e = v.dot(w0);
	double sc = (b * e - c * d) / (a * c - pow(b, 2));
	double tc = (a*e - b*d ) / (a * c - pow(b, 2));
	double dist = (u0 - v0 + sc * u - tc * v).norm();
	return dist;

}


void FourPCS::FindCongruent(double delta)
{
	double min_dist = 0;
	pcl::PointCloud<pcl::PointXYZ> B = this->Base;
	pcl::PointCloud<pcl::PointXYZ> Q = this->Q_cloud;
	Eigen::Vector3d p1, p2, p3, p4, b1, b2, b3, b4;
	p1 << B.points[0].x, B.points[0].y, B.points[0].z;
	p2 << B.points[1].x, B.points[1].y, B.points[1].z;
	p3 << B.points[2].x, B.points[2].y, B.points[2].z;
	p4 << B.points[3].x, B.points[3].y, B.points[3].z;
	double dist1 = Line2LineDist(p1, p2, p3, p4);
	double dist2 = Line2LineDist(p1, p3, p2, p4);
	double dist3 = Line2LineDist(p1, p4, p2, p3);
	b1 = p1;
	if (dist2 >= dist1 && dist3 >= dist1) {
		min_dist = dist1;
		b2 = p2;
		b3 = p3;
		b4 = p4;
	}
	else if (dist1 >= dist2 && dist3 >= dist2) {
		min_dist = dist2;
		b2 = p3;
		b3 = p2;
		b4 = p4;
	}
	else {
		min_dist = dist3;
		b2 = p4;
		b3 = p2;
		b4 = p3;
	}
	
	

}


//double FourPCS::EuclDist(pcl::PointXYZ p1, pcl::PointXYZ p2) {
//	return pow(double(p1.x - p2.x),2) + pow(double(p1.y - p2.y), 2) + pow(double(p1.z - p2.z), 2);
//}

//void FourPCS::LoadPointCloud() {
//	pcl::PointCloud<pcl::PointXYZ>::Ptr P_cloud_ptr;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr Q_cloud_ptr;
//	*Q_cloud_ptr = this->Q_cloud;
//	*P_cloud_ptr = this->P_cloud;
//	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filepath, *Q_cloud_ptr) == -1 || pcl::io::loadPLYFile<pcl::PointXYZ>(filepath, *P_cloud_ptr) == -1) {
//		std::cerr << "can't load point cloud file" << std::endl;
//	}
//}

