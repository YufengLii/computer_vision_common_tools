#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
//#include <Eigen/Dense>

//using namespace Eigen;
using namespace std;

//string pointcloud_dir_name = "pointcloud_feature\\";
string pointcloud_dir_name = "pointcloud\\";
string transformed_pointcloud_dir_name = "transformed_pointcloud\\";

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;


void generate_pointcloud(char filepath[], string plyfilename, Eigen::Matrix4d transform) {


	string file_ply = filepath + pointcloud_dir_name +plyfilename;
	std::cout << file_ply << endl;


	PointCloud source_cloud;
	PointCloud transformed_cloud;

	pcl::io::loadPLYFile(file_ply, source_cloud);
	pcl::transformPointCloud(source_cloud, transformed_cloud, transform);


	// 设置并保存点云
	transformed_cloud.height = 1;
	transformed_cloud.width = transformed_cloud.points.size();
	transformed_cloud.is_dense = false;

	try {
		//保存点云图
		pcl::io::savePLYFile(filepath + transformed_pointcloud_dir_name + plyfilename, transformed_cloud);
	}
	catch (pcl::IOException &e) {
		std::cout << e.what() << endl;
	}
	
}


int main(int argc, char** argv)
{

	string pointcloud_filenames = "pointcloud_filename.txt";
	string camera_poses_filename = "camera_poses.txt";

	ifstream rgb_depth_file(argv[1]+ pointcloud_filenames, ios::in);
	ifstream camera_poses(argv[1] + camera_poses_filename, ios::in);


	if (rgb_depth_file.is_open() && camera_poses.is_open())
	{
		std::cout << argv[1] << " open successed! " << endl;	
	}
	else
	{
		std::cout << argv[1] << " open failed! " << endl;
		return 0;
	}


	char buffer[20];
	char buffer2[200];

	string pointcloud_filename;

	//Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

	//double t1, t2, t3;
		//t1_before = 0, t2_before=0, t3_before = 0, q1_before = 0, q2_before = 0, q3_before = 0, q4_before = 1;
	//string timestamp;

	while (!rgb_depth_file.eof() || !camera_poses.eof())
	{
		rgb_depth_file.getline(buffer, sizeof(buffer));
		camera_poses.getline(buffer2, sizeof(buffer2));
		stringstream filenames(buffer);
		stringstream cur_pose(buffer2);

		filenames >> pointcloud_filename;
		std::cout << pointcloud_filename << endl;
		

		//Eigen::Matrix3d  Rotationmat;

		cur_pose >> transform(0, 0);
		cur_pose >> transform(0, 1);
		cur_pose >> transform(0, 2);
		cur_pose >> transform(0, 3);
		cur_pose >> transform(1, 0);
		cur_pose >> transform(1, 1);
		cur_pose >> transform(1, 2);
		cur_pose >> transform(1, 3);
		cur_pose >> transform(2, 0);
		cur_pose >> transform(2, 1);
		cur_pose >> transform(2, 2);
		cur_pose >> transform(2, 3);
		transform(3, 0)=0;
		transform(3, 1)=0;
		transform(3, 2) = 0;
		transform(3, 3) = 1;

		//cur_pose >> Rotationmat(0, 0);
		//cur_pose >> Rotationmat(1, 0);
		//cur_pose >> Rotationmat(2, 0);
		//
		//cur_pose >> t1;

		//cur_pose >> Rotationmat(0, 1);
		//cur_pose >> Rotationmat(1, 1);
		//cur_pose >> Rotationmat(2, 1);
		//cur_pose >> t2;
		//
		//cur_pose >> Rotationmat(0, 2);
		//cur_pose >> Rotationmat(1, 2);
		//cur_pose >> Rotationmat(2, 2);
		//

		//cur_pose >> t3;

		//transform.translation() << t1,t2,t3;
		//transform.rotate(Rotationmat);


		//Eigen::Quaterniond Q1(q1,q2,q3,q4);
		//Eigen::Quaterniond Q1_before(q1_before, q2_before, q3_before, q4_before);
		
		//Eigen::Matrix3d R1 = Q1.normalized().toRotationMatrix();
		//Eigen::Matrix3d R2 = Q1_before.normalized().toRotationMatrix();

		//Eigen::Matrix3d R4;
		//R4 = Q1.matrix();
		//Quaterniond Q2 = Q1.cast<float>();

		generate_pointcloud(argv[1], pointcloud_filename, transform);

		filenames >> pointcloud_filename;

		std::cout << pointcloud_filename << endl;


	}



	rgb_depth_file.close();
	camera_poses.close();
	return 0;
}