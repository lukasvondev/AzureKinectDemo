#include "SmothDepthImage.h"

cv::Mat smooth_depth_image(cv::Mat src, const int max_hole_size)
{
	int width = src.cols;
	int height = src.rows;
	int channels = src.channels();
	int types = src.type();

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
