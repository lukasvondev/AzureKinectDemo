#include <iostream>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>

cv::Mat convert16UTo8U(cv::Mat src);
cv::Mat smoothDepthImage(cv::Mat src, const int max_hole_size);
bool depthImageDiff(cv::Mat background, cv::Mat depth_image, cv::Mat* dst, uint16_t threshold = 0);

int main(int argc, char** argv)
{
	cv::Mat img1 = cv::imread(R"(C:\Users\Ding\Desktop\smoothed1.png)", cv::IMREAD_UNCHANGED);
	cv::Mat img2 = cv::imread(R"(C:\Users\Ding\Desktop\smoothed2.png)", cv::IMREAD_UNCHANGED);

	cv::Mat smoothed_img1 = smoothDepthImage(img1, 10);
	cv::Mat smoothed_img2 = smoothDepthImage(img2, 10);

	cv::Mat diff;
	if (depthImageDiff(smoothed_img1, smoothed_img2, &diff, 15))
	{
		cv::Mat smoothed_img1_U8 = convert16UTo8U(smoothed_img1);
		cv::Mat smoothed_img2_U8 = convert16UTo8U(smoothed_img2);
		cv::Mat dst = convert16UTo8U(diff);

		cv::imshow("smoothed_img1_U8", smoothed_img1_U8);
		cv::imshow("smoothed_img2_U8", smoothed_img2_U8);

		cv::imshow("absdiff result", dst);
		cv::waitKey();
		cv::imwrite(R"(C:\Users\Ding\Desktop\diff.png)", diff);
	}
	return EXIT_SUCCESS;
}
/// <summary>
/// Smooth depth image
/// </summary>
/// <param name="src">source depth image you want to smooth, data type uint16</param>
/// <param name="max_hole_size">max value of hole size that you want to make up</param>
/// <returns>smoothed depth image, data type uint16</returns>
cv::Mat smoothDepthImage(cv::Mat src, const int max_hole_size)
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
			if (data == 0)
			{
				mask.ptr<uint8_t>(row)[col] = 255;
			}
		}
	}

	// kernel
	cv::Mat kernel = cv::Mat::ones(max_hole_size, max_hole_size, CV_8UC1);

	cv::Mat erosion;
	cv::erode(mask, erosion, kernel);

	cv::absdiff(mask, erosion, mask);

	cv::Mat dst;
	cv::inpaint(src, mask, dst, static_cast<double>(max_hole_size), cv::INPAINT_NS);

	return dst;
}
/// <summary>
/// Convert image that data type is uint16 into uint8
/// </summary>
/// <param name="src">source depth image, data type uint16</param>
/// <returns>destination depth image, data type uint8</returns>
cv::Mat convert16UTo8U(cv::Mat src)
{
	cv::Mat tmp;
	cv::Mat dst = cv::Mat::zeros(src.rows, src.cols, CV_8U);
	cv::normalize(src, tmp, 0, 255, cv::NORM_MINMAX);
	cv::convertScaleAbs(tmp, dst);
	return dst;
}
/// <summary>
/// Difference between depth image and background. The shape between them must be same.
/// </summary>
/// <param name="background">background image</param>
/// <param name="depth_image">background image with foreground</param>
/// <param name="dst">result depth image</param>
/// <param name="threshold">threshold to adjust difference result</param>
/// <returns>the result(true or false) of difference between two images</returns>
bool depthImageDiff(cv::Mat background, cv::Mat depth_image, cv::Mat* dst, uint16_t threshold)
{
	if (background.cols != depth_image.cols || background.rows != depth_image.rows || background.channels() != depth_image.channels())
	{
		std::cout << "[ERROR] The shape of 2 input images is not same.";
		return false;
	}

	*dst = cv::Mat::zeros(background.rows, background.cols, CV_16U);

	for (int row = 0; row < background.rows; row++)
	{
		for (int col = 0; col < background.cols; col++)
		{
			uint16_t data_1 = background.ptr<uint16_t>(row)[col];
			uint16_t data_2 = depth_image.ptr<uint16_t>(row)[col];
			if (data_1 - data_2 > threshold)
			{
				dst->ptr<uint16_t>(row)[col] = data_2;
				//std::cout << data_2 << std::endl;
			}
		}
	}
	return true;
}