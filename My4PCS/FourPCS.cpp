#include "FourPCS.h"
#include<Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include<math.h>
#include<pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h> 
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
	cout << "Random Sample 1-3 Points" << endl;
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
	cout << "Selecting Fourth Point" << endl;
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
		if (!CheckLegal(p1-p2, p1-p3, p1-p4) || !CheckLegal(p2 - p1, p2 - p3, p2 - p4) || !CheckLegal(p3 - p1, p3 - p2, p3 - p4))//检查sc和tc是否在[0,1]
		{
			continue;
		}
		Eigen::Vector3d norm = VectorDecomp(p1-p2, p1-p3, p1-p4)[0];//获取第四点指向平面的向量
		double temp = Point2TriangleDist(p1, p2, p3, p4);//第四点到前三点构成三角形的距离
		if (max_dist < temp && norm.norm() < threh) {//点到平面距离小于阈值时，选择距离最大的那个
			norm_norm = norm.norm();
			max_dist = temp;
			best_point = p4;
		}
	}
	if (max_dist == 0)
		cerr << "未找到符合条件的基，请重试或调整参数" << endl;
	cout << "Final Distance to Plane" << norm_norm << endl;
	cout << "Final Distance to Trianglar " << max_dist << endl;
	//保存第四点
	Base.points[3].x = best_point[0];
	Base.points[3].y = best_point[1];
	Base.points[3].z = best_point[2];
}

bool FourPCS::CheckLegal(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {
	Eigen::Matrix3d A = Eigen::Matrix3d::Zero(3, 3);
	vector<Eigen::Vector3d> resault = VectorDecomp(p1, p2, p3);//向量分解为法向和正交投影
	Eigen::Vector2d b;
	b << resault[1][0], resault[1][1];
	A.col(0) = p1;
	A.col(1) = p2;
	Eigen::Matrix2d B = A.block<2,2>(0,0);// A.block<2, 2>(0, 0);
	Eigen::Vector2d lamda = B.colPivHouseholderQr().solve(b);//解方程求投影在基下的线性表示
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
	//用叉乘算三角形面积得到距离
	dist = (x - y).cross(x - w).norm()/ (x - y).norm();
	if (((y - z).cross(y - w).norm())/ (y - z).norm() > dist)
		dist = ((y - z).cross(y - w).norm()) / (y - z).norm();
	if (((z - x ).cross(z - w).norm())/ (z - x).norm() > dist)
		dist = ((z - x).cross(z - w).norm()) / (z - x).norm();
	return dist;
}

double FourPCS::Line2LineDist(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1) {
	//向量法求两条线距离
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

void FourPCS::Line2LineLeastSquareIntersection(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1, double *r1, double *r2) {
	//向量法求两条线距离
	double a, b, c, d, e;
	Eigen::Vector3d u = u1 - u0;
	Eigen::Vector3d v = v1 - v0;
	Eigen::Vector3d w0 = u0 - v0;
	a = u.dot(u);
	b = u.dot(v);
	c = v.dot(v);
	d = u.dot(w0);
	e = v.dot(w0);
	*r1 = (b * e - c * d) / (a * c - pow(b, 2));
	*r2 = (a * e - b * d) / (a * c - pow(b, 2));
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
	//寻找两条线距离最小的点对
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
	//记录线段交比
	double* r1 = nullptr;
	double* r2 = nullptr;
	Line2LineLeastSquareIntersection(b1, b2, b3, b4, r1, r2);
	//线段长度特征
	double d1 = (b1 - b2).norm();
	double d2 = (b3 - b4).norm();

	pcl::PointCloud<pcl::PointXYZ> R1, R2, PAI1, PAI2;
	//搜索近似长度点对
	cout << "Matching Pairs from source" << endl;
	R1 = FindMatchPairs(P_cloud, d1, 0.02);
	cout << "R1 Size: " << R1.size()<< endl;
	cout << "Matching Pairs from target" << endl;
	R2 = FindMatchPairs(Q_cloud, d2, 0.02);
	cout << "R2 Size: " << R2.size() << endl;
	//按照交比猜测可能的交点
	for (int i = 0; i < R1.size(); i = i + 2) {
		PAI1.push_back(R1.points[i] + *r1 * NormalizePointXYZ(R1.points[i]));
		PAI1.push_back(R1.points[i+1] + *r1 * NormalizePointXYZ(R1.points[i+1]));
	}
	for (int i = 0; i < R2.size(); i = i + 2) {
		PAI2.push_back(R2.points[i] + *r1 * NormalizePointXYZ(R2.points[i]));
		PAI2.push_back(R2.points[i + 1] + *r1 * NormalizePointXYZ(R2.points[i + 1]));
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//建立KDTree，对PAI1中所有点，搜索PAI2的最近邻

}
pcl::PointCloud<pcl::PointXYZ> FourPCS::FindMatchPairs(pcl::PointCloud<pcl::PointXYZ> &cloud, double d, double delta) {
	pcl::PointCloud<pcl::PointXYZ> candidatep_points;
	for (int i = 0; i < cloud.size(); i++) {
		if (cloud.size() % (cloud.size() / 100) == 0)
			cout << "." << endl;
		Eigen::Vector3d pi; 
		pi << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
		for (int j = i; j < cloud.size(); j++) {
			Eigen::Vector3d pj;
			pj << cloud.points[j].x, cloud.points[j].y, cloud.points[j].z;
			double r = (pi - pj).norm();
			if (abs(r - d) < delta) {
				candidatep_points.push_back(cloud.points[i]);
				candidatep_points.push_back(cloud.points[j]);
			}
		}
	}
	return candidatep_points;
}

pcl::PointXYZ FourPCS::NormalizePointXYZ(pcl::PointXYZ p) {
	Eigen::Vector3d ep;
	pcl::PointXYZ pclp;
	ep << p.x, p.y, p.z;
	ep.normalize();
	pclp.x = ep[0];
	pclp.y = ep[1];
	pclp.z = ep[2];
	return pclp;
}

pcl::PointXYZ operator*(double k, pcl::PointXYZ p) {
	pcl::PointXYZ res = p;
	res.x = p.x * k;
	res.y = p.y * k;
	res.z = p.z * k;
	return res;

}

pcl::PointXYZ operator+(pcl::PointXYZ p1, pcl::PointXYZ p2) {
	pcl::PointXYZ res = p1;
	res.x = p1.x + p2.x;
	res.y = p1.y + p2.y;
	res.z = p1.z + p2.z;
	return res;
}

pcl::PointXYZ operator-(pcl::PointXYZ p1, pcl::PointXYZ p2) {
	pcl::PointXYZ res = p1;
	res.x = p1.x - p2.x;
	res.y = p1.y - p2.y;
	res.z = p1.z - p2.z;
	return res;
}