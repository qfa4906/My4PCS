#include <iostream>
#include<string>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
using namespace std;
#include "FourPCS.h"
void savePointCloud(pcl::PointCloud<pcl::PointXYZ> cloud, std::string outpath);
void grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

	pcl::VoxelGrid<pcl::PointXYZ> sor;     // 创建滤波对象
	sor.setInputCloud(cloud);              // 给滤波对象设置需要过滤的点云
	sor.setLeafSize(0.5f, 0.5f, 0.5f);  // 设置滤波时创建的体素大小为1cm立方体
	sor.filter(*cloud_filtered);           // 执行滤波处理，存储输出到cloud_filtered
	return;
}
int main()
{
	string filepath1 = "mesh_2-3-4_node.ply";
	string filepath2 = "mesh_2-3-4_node_trans.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Q_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Q_cloud_transformed_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Q_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filepath1, *P_cloud_ptr) == -1 || pcl::io::loadPLYFile<pcl::PointXYZ>(filepath2, *Q_cloud_ptr) == -1) {
		std::cerr << "can't load point cloud file" << std::endl;
	}
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	transformation_matrix << 0.8, -0.6, 0, 0,
							0.6, 0.8, 0,0,
							0, 0, 1,0,
							0, 0, 0, 1;

	pcl::transformPointCloud(*Q_cloud_ptr, *Q_cloud_transformed_ptr, transformation_matrix);
	grid_filter(P_cloud_ptr, P_cloud_ptr);
	grid_filter(Q_cloud_transformed_ptr, Q_cloud_transformed_ptr);
	FourPCS* pcs = new FourPCS(*P_cloud_ptr,*Q_cloud_transformed_ptr);
	pcs->SelectCoplanarBase();
	pcs->FindCongruent(0.01);
	
	savePointCloud(pcs->Base, "Base.ply");
	savePointCloud(pcs->congruent_base, "CBase.ply");
	savePointCloud(pcs->save_cloud, "save_cloud.ply");
	savePointCloud(pcs->save_cloud2, "save_cloud2.ply");
	savePointCloud(*Q_cloud_transformed_ptr, "Q_cloud_ptr.ply");
	

}

void savePointCloud(pcl::PointCloud<pcl::PointXYZ> cloud, std::string outpath)
{
	std::cerr << "save path is :" << outpath << endl;
	//将string保存路径转为char*
	char* path = new char[outpath.size() + 1];
	strcpy(path, outpath.c_str());
	std::cerr << "Path is : " << path << " ." << std::endl;

	//写出点云图
	pcl::PLYWriter writer;
	writer.write(path, cloud, true);
	//std::cerr << "PointCloud has : " << cloud->width * cloud->height << " data points." << std::endl;
}



