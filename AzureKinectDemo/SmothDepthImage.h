#pragma once

#include <opencv2/opencv.hpp>

cv::Mat smooth_depth_image(cv::Mat src, const int max_hole_size);
