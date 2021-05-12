/*
* rgb image + transformed depth image -> point cloud image
* 根据彩色图和深度图生成点云
*/
#pragma warning(disable:4996)
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include <k4a/k4a.hpp>

#include <pcl/io/ply_io.h>

#include <pcl/impl/point_types.hpp>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace cv;

k4a::calibration readCalibrationFromFile(const string file_path);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBD2Cloud(const k4a::image& color_img, const k4a::image& transformed_depth_img);

int main(int argc, char** argv)
{
	string cloud_save_path = "model.ply";
	if (argc != 3)
	{
		cout << "[Error]:Please enter as following format:" << endl;
		cout << "PointCloudDemo.exe [color image path] [depth image path]" << endl;
		return 1;
	}

	string color_image_path = argv[1];
	string depth_image_path = argv[2];

	Mat color_frame = imread(color_image_path, IMREAD_UNCHANGED);

	if (color_frame.empty() || color_frame.depth() != CV_8U)
	{
		cerr << "[WARNING]:Cannot read color image. No such a file, or the image format is not CV_8U." << endl;
	}

	Mat depth_frame = imread(depth_image_path, IMREAD_ANYDEPTH);

	if (depth_frame.empty() || depth_frame.depth() != CV_16U)
	{
		cerr << "[WARNING]:Cannot read depth image. No such a file, or the image format is not CV_16U." << endl;
	}

	k4a::image color_image = NULL;
	k4a::image depth_image = NULL;

	color_image = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
		color_frame.size().width,
		color_frame.size().height,
		color_frame.size().width * 4 * (int)sizeof(uint8_t));
	memcpy(color_image.get_buffer(),
		color_frame.data,
		color_frame.size().width * color_frame.size().height * 4 * (int)sizeof(uint8_t));

	depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
		depth_frame.size().width,
		depth_frame.size().height,
		depth_frame.size().width * (int)sizeof(uint16_t));
	memcpy(depth_image.get_buffer(),
		depth_frame.data,
		depth_frame.size().width * depth_frame.size().height * (int)sizeof(uint16_t));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	/*
	//读取.ply格式点云文件
	if (pcl::io::loadPLYFile<pcl::PointXYZ>("C:/Users/Ding/Desktop/bunny/reconstruction/bun_zipper.ply", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		system("PAUSE");
		return (-1);
	}
	*/

	// 保存点云数据

	cloud = RGBD2Cloud(color_image, depth_image);
	pcl::visualization::CloudViewer viewer("Viewer");
	viewer.showCloud(cloud);

	pcl::io::savePLYFile(cloud_save_path, *cloud);

	system("PAUSE");

	return 0;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBD2Cloud(const k4a::image& color_img, const k4a::image& transformed_depth_img)
{
	int color_image_width = color_img.get_width_pixels();
	int color_image_height = color_img.get_height_pixels();

	string path = "E:/vsProject/AzureKinectDemo/AzureKinectDemo/calibration.json";

	k4a::calibration calibration = readCalibrationFromFile(path);
	k4a::transformation transformation = k4a::transformation(calibration);

	const k4a_calibration_intrinsic_parameters_t* intrinsic_color = &calibration.color_camera_calibration.intrinsics.parameters;

	constexpr float kMillmeter2Meter = 1.0 / 1000.0f;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width = color_image_width;
	cloud->height = color_image_height;
	cloud->is_dense = false;
	cloud->resize(color_image_width * color_image_height);

	const uint16_t* depth_buffer = reinterpret_cast<const uint16_t*>(transformed_depth_img.get_buffer());
	const uint8_t* color_buffer = color_img.get_buffer();

	// 相机参数
	float fx = intrinsic_color->param.fx;
	float fy = intrinsic_color->param.fy;
	float cx = intrinsic_color->param.cx;
	float cy = intrinsic_color->param.cy;

	for (int y = 0, idx = 0; y < color_image_height; y++)
	{
		for (int x = 0; x < color_image_width; x++, idx++)
		{
			pcl::PointXYZRGB point;
			float depth = static_cast<float>(depth_buffer[idx]);
			uint8_t a = color_buffer[4 * idx + 3];
			if (depth <= .0f || a == 0)
			{
				//point.x = point.y = point.z = numeric_limits<float>::quiet_NaN();
				point.x = point.y = point.z = .0;
				point.r = point.g = point.b = 0;
			}
			else
			{
				point.x = kMillmeter2Meter * (x - cx) * depth / fx;
				point.y = kMillmeter2Meter * (y - cy) * depth / fy;
				point.z = kMillmeter2Meter * depth;
				point.r = color_buffer[4 * idx + 2];
				point.g = color_buffer[4 * idx + 1];
				point.b = color_buffer[4 * idx + 0];
			}

			cloud->points[idx] = point;
		}
	}
	return cloud;
}

k4a::calibration readCalibrationFromFile(const string file_path)
{
	basic_ifstream<uint8_t> fin(file_path);
	auto eos = istreambuf_iterator<uint8_t>();
	auto buffer = vector<uint8_t>(istreambuf_iterator<uint8_t>(fin), eos);
	k4a::calibration calibration;
	calibration = k4a::calibration::get_from_raw(buffer.data(),
		buffer.size(),
		K4A_DEPTH_MODE_WFOV_2X2BINNED,
		K4A_COLOR_RESOLUTION_720P);
	return calibration;
}