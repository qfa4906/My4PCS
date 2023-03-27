#include <iostream>
#include<string>
using namespace std;
#include "FourPCS.h"
void savePointCloud(pcl::PointCloud<pcl::PointXYZ> cloud, std::string outpath);
int main()
{
	string filepath1 = "mesh_2-3-4_node.ply";
	string filepath2 = "mesh_2-3-4.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr P_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Q_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filepath1, *P_cloud_ptr) == -1 || pcl::io::loadPLYFile<pcl::PointXYZ>(filepath2, *Q_cloud_ptr) == -1) {
		std::cerr << "can't load point cloud file" << std::endl;
	}
	FourPCS* pcs = new FourPCS(*P_cloud_ptr,*Q_cloud_ptr);
	pcs->SelectCoplanarBase();
	pcs->FindCongruent(0.1);
	savePointCloud(pcs->Base, "Base.ply");


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



