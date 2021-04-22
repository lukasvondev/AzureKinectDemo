/*
* 1.RGB图像，Depth图像查看器
*
* 2.按下Space保存RGB图像，深度图像和矫正配准后的深度图像
*
* 格式：
*		RGB Image: BGRA C8_U4
*		Depth Image: BGRA C8_U4
*		Transformed Depth Image: uint_16 C16_U1
*
* 3.存储相机参数到.json文件中
*/
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>

#include <opencv2/opencv.hpp>
#include <k4a/k4a.hpp>

#include "k4aPixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"

using namespace std;
using namespace cv;

string GetSystemTime();

int main(int argc, char** argv) {
	// 获取已连接设备数
	const uint32_t device_count = k4a::device::get_installed_count();
	if (device_count == 0)
	{
		cout << "no azure kinect devices detected." << endl;
		return 1;
	}

	// 配置Kinect参数
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;

	// 打开设备
	cout << "Start opening azure kinect device..." << endl;
	k4a::device dev = k4a::device::open(K4A_DEVICE_DEFAULT);
	dev.start_cameras(&config);
	cout << "Finished opening azure kinect device..." << endl;

	// 畸变矫正参数
	k4a::calibration calibration = dev.get_calibration(config.depth_mode, config.color_resolution);
	k4a::transformation transformation = k4a::transformation(calibration);

	vector<uint8_t> raw_calibration = dev.get_raw_calibration();
	string raw_calibration_buffer(raw_calibration.begin(), raw_calibration.end());

	// 写入相机参数到json文件中
	ofstream fout("calibration.json");
	fout << raw_calibration_buffer;

	// capture
	k4a::capture capture;

	// frame
	k4a::image depth_image;
	k4a::image color_image;
	k4a::image irImage;
	k4a::image transformed_depth_image;

	// Opencv frame
	Mat depth_frame;
	Mat color_frame;
	Mat irFrame;
	Mat transformed_depth_frame;

	int key = NULL;

	int color_image_width = NULL;
	int color_image_height = NULL;

	uint8_t* colorTextureBuffer;
	uint16_t* transformedDepthTextureBuffer;

	// 动态数组depthTextureBuffer 每个元素为Pixel
	vector<Pixel> depthTextureBuffer;

	// 存储路径
	const string path = "./Image";
	const string depth_image_path = path + "/Depth_Image/";
	const string color_image_path = path + "/RGB_Image/";
	const string transformed_depth_image_path = path + "/Transformed_Depth_Raw_Data/";
	const string suffix = ".png";

	while (true)
	{
		if (dev.get_capture(&capture, chrono::milliseconds(0)))
		{
			color_image = capture.get_color_image();
			depth_image = capture.get_depth_image();
			// irImage = capture.get_ir_image();

			// color
			colorTextureBuffer = color_image.get_buffer();
			color_frame = Mat(color_image.get_height_pixels(), color_image.get_width_pixels(), CV_8UC4, colorTextureBuffer);

			// depth
			sip::ColorizeDepthImage(depth_image,
				dpc::DepthPixelColorizer::ColorizeBlueToRed,
				sip::GetDepthModeRange(config.depth_mode),
				&depthTextureBuffer);
			depth_frame = Mat(depth_image.get_height_pixels(), depth_image.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());

			color_image_width = color_image.get_width_pixels();
			color_image_height = color_image.get_height_pixels();

			//transformed depth image
			transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
				color_image_width,
				color_image_height,
				color_image_width * (int)sizeof(uint16_t));

			transformation.depth_image_to_color_camera(depth_image, &transformed_depth_image);
			transformedDepthTextureBuffer = reinterpret_cast<uint16_t*>(transformed_depth_image.get_buffer());
			
			transformed_depth_frame = Mat(transformed_depth_image.get_height_pixels(),
				transformed_depth_image.get_width_pixels(),
				CV_16U,
				transformedDepthTextureBuffer);

			imshow("color", color_frame);
			imshow("depth", depth_frame);

			key = waitKey(10);

			if (key == 27)
			{
				color_frame.release();
				depth_frame.release();
				transformed_depth_frame.release();
				dev.close();
				break;
			}
			if (key == 32)
			{
				string dt = GetSystemTime();
				imwrite(color_image_path + dt + suffix, color_frame);
				imwrite(depth_image_path + dt + suffix, depth_frame);
				imwrite(transformed_depth_image_path + dt + suffix, transformed_depth_frame);
				cout << "Image saved." << endl;
			}
		}
	}
	destroyAllWindows();
	return 0;
}

// 获取当前系统时间
inline string GetSystemTime()
{
	time_t tt = chrono::system_clock::to_time_t(chrono::system_clock::now());
	struct tm ptm;
	localtime_s(&ptm, &tt);
	string year = to_string(ptm.tm_year + 1900);
	string month = to_string(ptm.tm_mon + 1);
	string day = to_string(ptm.tm_mday);
	string hour = to_string(ptm.tm_hour);
	string minute = to_string(ptm.tm_min);
	string second = to_string(ptm.tm_sec);
	string date = year + "_" + month + "_" + day + "_" + hour + "_" + minute + "_" + second;
	return date;
}