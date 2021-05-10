/*
* ===========================
* 对深度图像进行空洞填充 滤波去噪
* ===========================
*/

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

cv::Mat smooth_depth_image(cv::Mat src, const int max_hole_size);

int main(int argc, char** argv)
{
	std::string file_path;
	std::cout << "Please Enter Depth Image Path:";
	std::cin >> file_path;

	cv::Mat depth = cv::imread(file_path, cv::IMREAD_UNCHANGED);

	cv::Mat smoothed_depth = smooth_depth_image(depth, 10);

	bool save_status = cv::imwrite("smoothed.png", smoothed_depth);

	if (save_status)
	{
		std::cout << "Image saved.";
	}

	return EXIT_SUCCESS;
}

cv::Mat smooth_depth_image(cv::Mat src, const int max_hole_size)
{
	int width = src.cols;
	int height = src.rows;
	int channels = src.channels();
	int types = src.type();

	std::cout << "Image Width: " << width << std::endl;
	std::cout << "Image Height: " << height << std::endl;
	std::cout << "Image Channels: " << channels << std::endl;
	std::cout << "Image Type: " << types << " Bytes" << std::endl;

	cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);

	for (int row = 0; row < height; row++)
	{
		for (int col = 0; col < width; col++)
		{
			uint16_t data = src.ptr<uint16_t>(row)[col];
			// cout << data << endl;
			if (data == 0)
			{
				mask.ptr<uint8_t>(row)[col] = 255;
				// cout << mask.ptr<uint8_t>(row)[col] << endl;
			}
		}
	}

	cv::Mat kernel = cv::Mat::ones(max_hole_size, max_hole_size, CV_8UC1);

	cv::Mat erosion;
	cv::erode(mask, erosion, kernel);

	cv::absdiff(mask, erosion, mask);

	cv::Mat dst;
	cv::inpaint(src, mask, dst, static_cast<double>(max_hole_size), cv::INPAINT_NS);

	return dst;
}