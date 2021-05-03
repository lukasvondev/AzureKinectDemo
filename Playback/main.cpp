/*
* Playback
*
*/
#include <iostream>
#include <string>
#include <vector>

#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <turbojpeg.h>

#include "k4aPixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"

// cvtMJPG2BGRA
bool cvtMJPG2BGRA(const k4a::image& mjpg_image, k4a::image& bgra_image);

k4a::playback playback;
k4a::capture capture;
k4a::calibration calibration;
k4a::transformation transformation;

int main(int argc, char** argv)
{
	std::string file_path;
	std::cout << "Please enter recording path: ";
	std::cin >> file_path;

	playback = k4a::playback::open(file_path.c_str());
	calibration = playback.get_calibration();
	transformation = k4a::transformation(calibration);

	bool stream_result = false;

	k4a::image color_image;
	k4a::image depth_image;

	cv::Mat color_frame;
	cv::Mat depth_frame;

	std::vector<Pixel> depthTextureBuffer;
	uint8_t* colorTextureBuffer;

	int key = NULL;

	while (true)
	{
		stream_result = playback.get_next_capture(&capture);
		if (stream_result)
		{
			k4a::image compressed_color_image = capture.get_color_image();
			cvtMJPG2BGRA(compressed_color_image, color_image);
			depth_image = capture.get_depth_image();

			colorTextureBuffer = color_image.get_buffer();

			sip::ColorizeDepthImage(depth_image,
				dpc::DepthPixelColorizer::ColorizeBlueToRed,
				sip::GetDepthModeRange(K4A_DEPTH_MODE_WFOV_2X2BINNED),
				&depthTextureBuffer);

			color_frame = cv::Mat(color_image.get_height_pixels(),
				color_image.get_width_pixels(),
				CV_8UC4,
				colorTextureBuffer);
			depth_frame = cv::Mat(depth_image.get_height_pixels(),
				depth_image.get_width_pixels(),
				CV_8UC4,
				depthTextureBuffer.data());

			cv::imshow("color image", color_frame);
			cv::imshow("depth image", depth_frame);
		}
		else
		{
			playback.close();
			break;
		}

		key = cv::waitKey(10);
		if (key == 27)
		{
			playback.close();
			break;
		}
	}
	return EXIT_SUCCESS;
}

inline bool cvtMJPG2BGRA(const k4a::image& mjpg_image, k4a::image& bgra_image)
{
	bgra_image = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
		mjpg_image.get_width_pixels(),
		mjpg_image.get_height_pixels(),
		mjpg_image.get_width_pixels() * 4 * (int)sizeof(uint8_t));
	tjhandle m_decompressor;
	m_decompressor = tjInitDecompress();

	const int decompressStatus = tjDecompress2(m_decompressor,
		mjpg_image.get_buffer(),
		static_cast<unsigned long>(mjpg_image.get_size()),
		bgra_image.get_buffer(),
		mjpg_image.get_width_pixels(),
		0,
		mjpg_image.get_height_pixels(),
		TJPF_BGRA,
		TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE);

	(void)tjDestroy(m_decompressor);

	return true;
}