#include "FourPCS.h"
#include<Eigen/Dense>
#include<iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <tbb/task_arena.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include<math.h>
#include<pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h> 
int THREAD_NUM = 1;
using namespace std;
FourPCS::FourPCS(pcl::PointCloud<pcl::PointXYZ> cloud1, pcl::PointCloud<pcl::PointXYZ> cloud2) {
	P_cloud = cloud1;
	Q_cloud = cloud2;
	cout << "P size:" << P_cloud.size();
	cout << "Q size:" << Q_cloud.size();

}

inline double DistPointXYZ(pcl::PointXYZ p1, pcl::PointXYZ p2) {
	return (pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
}

void FindMatchPairsThread(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::PointXYZ>& R1, pcl::PointCloud<pcl::PointXYZ>& R2, double delta1, double delta2, double d, int a, int b) {
	double d2 = pow(d, 2);
	cout << "this is thread: " << a <<endl;
	for (int i = a; i < cloud.size(); i += THREAD_NUM) {
		if (i % 100 == 0)
			cout << i << endl;
		for (int j = i; j < cloud.size(); j++) {
			double dist =pow(DistPointXYZ(cloud.points[i], cloud.points[j]),0.5);
			if ((dist - delta1) < d2 && (dist - delta1) > -d2) {
				R1.push_back(cloud.points[i]);
				R1.push_back(cloud.points[j]);
			}
			if ((dist - delta2) < d2 && (dist - delta2) > -d2) {
				R2.push_back(cloud.points[i]);
				R2.push_back(cloud.points[j]);
			}
		}
	}
}

pcl::PointXYZ operator*(double k, pcl::PointXYZ p) {
	pcl::PointXYZ res;
	res.x = p.x * k;
	res.y = p.y * k;
	res.z = p.z * k;
	return res;

}

pcl::PointXYZ operator+(pcl::PointXYZ p1, pcl::PointXYZ p2) {
	pcl::PointXYZ res;
	res.x = p1.x + p2.x;
	res.y = p1.y + p2.y;
	res.z = p1.z + p2.z;
	return res;
}

pcl::PointXYZ operator-(pcl::PointXYZ& p1, pcl::PointXYZ& p2) {
	pcl::PointXYZ res;
	res.x = p1.x - p2.x;
	res.y = p1.y - p2.y;
	res.z = p1.z - p2.z;
	return res;
}
void FourPCS::SelectCoplanarBase() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud = P_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sample(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_sample->width = 4;
	cloud_sample->height = 1;
	double MaxArea = 0;
	cout << "Random Sample 1-3 Points" << endl;
	for (int i = 0; i < 100; i++) {//�������100�� ѡ��Χ���������������
		pcl::RandomSample<pcl::PointXYZ> rs;
		rs.setInputCloud(cloud);
		//��������������   
		rs.setSample(4);
		//�²����������cloud_sample
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
	for (int j = 0; j < cloud->size();j++) {//�������ƣ�ѡ�����ŵ��ĵ�
		if (j % 1000 == 0)
			cout << "max_dist: " << max_dist << "norm: " << norm_norm << endl;
		Eigen::Vector3d p1(3), p2(3), p3(3), p4(3);
		p1 << Base.points[0].x, Base.points[0].y, Base.points[0].z;
		p2 << Base.points[1].x, Base.points[1].y, Base.points[1].z;
		p3 << Base.points[2].x, Base.points[2].y, Base.points[2].z;
		p4 << cloud->points[j].x, cloud->points[j].y, cloud->points[j].z;//ѡȡ�ĵ��ĵ�
		if (p1 == p4 || p2 == p4 || p3 == p4)
		{
			continue;
		}
		if (!CheckLegal(p1-p2, p1-p3, p1-p4) || !CheckLegal(p2 - p1, p2 - p3, p2 - p4) || !CheckLegal(p3 - p1, p3 - p2, p3 - p4))//���sc��tc�Ƿ���[0,1]
		{
			continue;
		}
		Eigen::Vector3d norm = VectorDecomp(p1-p2, p1-p3, p1-p4)[0];//��ȡ���ĵ�ָ��ƽ�������
		double temp = Point2TriangleDist(p1, p2, p3, p4);//���ĵ㵽ǰ���㹹�������εľ���
		if (max_dist < temp && norm.norm() < threh) {//�㵽ƽ�����С����ֵʱ��ѡ����������Ǹ�
			norm_norm = norm.norm();
			max_dist = temp;
			best_point = p4;
		}
	}
	if (max_dist == 0)
		cerr << "δ�ҵ����������Ļ��������Ի��������" << endl;
	cout << "Final Distance to Plane" << norm_norm << endl;
	cout << "Final Distance to Trianglar " << max_dist << endl;
	//������ĵ�
	Base.points[3].x = best_point[0];
	Base.points[3].y = best_point[1];
	Base.points[3].z = best_point[2];
}

bool FourPCS::CheckLegal(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {
	Eigen::Matrix3d A = Eigen::Matrix3d::Zero(3, 3);
	vector<Eigen::Vector3d> resault = VectorDecomp(p1, p2, p3);//�����ֽ�Ϊ���������ͶӰ
	Eigen::Vector2d b;
	b << resault[1][0], resault[1][1];
	A.col(0) = p1;
	A.col(1) = p2;
	Eigen::Matrix2d B = A.block<2,2>(0,0);// A.block<2, 2>(0, 0);
	Eigen::Vector2d lamda = B.colPivHouseholderQr().solve(b);//�ⷽ����ͶӰ�ڻ��µ����Ա�ʾ
	if (lamda[0] * lamda[1] > 0 && lamda[0] + lamda[1] < 1)
		return false;
	else
		return true;
}


vector<Eigen::Vector3d> FourPCS::VectorDecomp(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z) {
	vector<Eigen::Vector3d> value(2);
	Eigen::Vector3d norm = x.cross(y).normalized();
	Eigen::Vector3d normalized_norm = norm.dot(z) * norm;//��xyƽ�淨��
	Eigen::Vector3d projection = z - normalized_norm;//��z��xyͶӰ
	value[0] = normalized_norm;
	value[1] = projection;
	return value;
}

double FourPCS::Point2TriangleDist(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z, Eigen::Vector3d w) {
	double dist = 0,temp = 0;
	//�ò��������������õ�����
	dist = (x - y).cross(x - w).norm()/ (x - y).norm();
	if (((y - z).cross(y - w).norm())/ (y - z).norm() > dist)
		dist = ((y - z).cross(y - w).norm()) / (y - z).norm();
	if (((z - x ).cross(z - w).norm())/ (z - x).norm() > dist)
		dist = ((z - x).cross(z - w).norm()) / (z - x).norm();
	return dist;
}

double FourPCS::Line2LineDist(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1) {
	//�������������߾���
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
	if( sc > 0 && tc > 0 && sc < 1 && tc < 1)
		return (u0 - v0 + sc * u - tc * v).norm();
	else
		return 0;
}

void FourPCS::Line2LineLeastSquareIntersection(Eigen::Vector3d u0, Eigen::Vector3d u1, Eigen::Vector3d v0, Eigen::Vector3d v1, double *r1, double *r2) {
	//�������������߾���
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
	//Ѱ�������߾�����С�ĵ��
	double dist1 = Line2LineDist(p1, p2, p3, p4);
	double dist2 = Line2LineDist(p1, p3, p2, p4);
	double dist3 = Line2LineDist(p1, p4, p2, p3);
	b1 = p1;
	if (dist1 != 0) { 
		min_dist = dist1;
		b2 = p2;
		b3 = p3;
		b4 = p4;
	}
	else if (dist2 != 0) {
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
	//��¼�߶ν���
	double* r1 = new double;
	double* r2 = new double;
	Line2LineLeastSquareIntersection(b1, b2, b3, b4, r1, r2);
	//�߶γ�������
	double d1 = (b1 - b2).norm();
	double d2 = (b3 - b4).norm();

	pcl::PointCloud<pcl::PointXYZ> R1, R2, PAI1, PAI2;//R1,R2��Լ���PAI1, PAI2���㼯��R�е�i��iΪż���͵�i+1�������һ����ԣ���ӦPAI�еĵ�i��iΪż���͵�i+1����Ϊ�������㡣
	//�������Ƴ��ȵ��
	cout << "Matching Pairs from source" << endl;
	//FindMatchPairs(Q_cloud,R1,R2,d1,d2, 0.02);
	FindMatchPairsThread(Q_cloud, R1, R2, d1, d2, 0.02, 0, 0);
	cout << "R1 Size: " << R1.size()<< endl;
	cout << "R2 Size: " << R2.size() << endl;
	//���ս��Ȳ²���ܵĽ���
	for (int i = 0; i < R1.size(); i = i + 2) {
		PAI1.push_back( R1.points[i] + *r1 * (R1.points[i + 1] + double(-1) * R1.points[i]) );
		PAI1.push_back(R1.points[i+1] + *r1 *(R1.points[i] + (-1) * R1.points[i + 1]));
	}
	for (int i = 0; i < R2.size(); i = i + 2) {
		PAI2.push_back(R2.points[i] + *r2 * (R2.points[i + 1] + (-1) * R2.points[i]));
		PAI2.push_back(R2.points[i + 1] + *r2 * (R2.points[i] + (-1) * R2.points[i + 1]));
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//����KDTree����PAI1�����е㣬����PAI2�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr PAI1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	*PAI1_ptr = PAI1;
	kdtree.setInputCloud(PAI1_ptr);
	pcl::PointXYZ pSearch, pMin, pMax;       //�����㣬����������ֵ����Сֵ
	pcl::getMinMax3D(*PAI1_ptr, pMin, pMax);    //��Ҫinclude<pcl/common/common.h>
	pcl::PointXYZ tmp;       //���ڴ洢��ʱ��
	float r = min_dist * 2;
	vector<int> PAI1_index;   //�洢��������
	vector<int> search_index;
	vector<float> ptRadius;      //�洢���ڶ�Ӧ�����ƽ��
	vector<int> PAI2_index;
	for (int k = 0; k < PAI2.size(); k++) {
		kdtree.radiusSearch(PAI2.points[k], r, search_index, ptRadius);
		if (search_index.size() > 0) {
			for (int i = 0; i < search_index.size(); i++) {
				PAI2_index.push_back(k);
				PAI1_index.push_back(search_index[i]);
			}
			
		}
	}
	cout << "search res:" << endl;
	cout << PAI1_index.size() << endl;
	cout << PAI2_index.size() << endl;
	
	for (size_t i = 0; i < PAI1_index.size(); ++i) {
		int pair_index1 = PAI1_index[i] / 2;
		int pair_index2 = PAI2_index[i] / 2;
		pcl::PointXYZ c1(R1[2*pair_index1]), c2(R1[2 * pair_index1 + 1]), c3(R2[2 * pair_index2]), c4(R2[2 * pair_index2 + 1]);
		congruent_base.push_back(c1);
		congruent_base.push_back(c2);
		congruent_base.push_back(c3);
		congruent_base.push_back(c4);
	}
	save_cloud = PAI1;
	save_cloud2 = PAI2;
}
void FourPCS::FindMatchPairs(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::PointXYZ>& R1, pcl::PointCloud<pcl::PointXYZ>& R2, double delta1, double delta2, double d) {
	vector<thread> threads(THREAD_NUM);
	for (int i = 0; i < THREAD_NUM; i++) {
		threads[i] = thread(FindMatchPairsThread, cloud, std::ref(R1), std::ref(R2), delta1, delta2, d, i, 0);
	}
	for (int i = 0; i < THREAD_NUM; i++) {
		threads[i].join();
	}

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




