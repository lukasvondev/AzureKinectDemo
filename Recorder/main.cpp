#include <k4a/k4a.hpp>
#include <k4arecord/record.hpp>

#include <iostream>
#include <chrono>
#include <string>

std::string GetSystemTime();

int main(int argc, char** argv)
{
	// 录制时长（单位：s）
	int record_length;
	
	// 存放路径
	std::string path = _pgmptr;
	size_t n = path.rfind("\\");
	std::string path_ = path.substr(0, n + 1);

	std::string time = GetSystemTime();
	
	std::string file_name = path_ + time + "_record.mkv";

	std::cout << file_name << std::endl;

	std::cout << "Please enter recording time(seconds): ";

	while (!(std::cin >> record_length))
	{
		std::cin.clear();
		while (std::cin.get() != '\n')
		{
			continue;
		}
		std::cout << "[Error]: Data type is not <int>! "
			<< "Please enter recording time(seconds): ";
	}
	
	k4a_device_configuration_t config;
	k4a::device device;
	k4a::capture capture;

	const int device_count = k4a::device::get_installed_count();
	if (device_count == 0)
	{
		std::cout << "No azure kinect device detected." << std::endl;
		return 1;
	}

	config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;

	std::cout << "Started opening K4A device..." << std::endl;
	device = k4a::device::open(K4A_DEVICE_DEFAULT);
	device.start_cameras(&config);
	std::cout << "Finished opening K4A device." << std::endl;

	k4a::record recording;

	recording = k4a::record::create(file_name.c_str(), device, config);
	recording.write_header();

	std::cout << "Started recording..." << std::endl;

	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

	while (true)
	{
		device.get_capture(&capture, std::chrono::milliseconds{ K4A_WAIT_INFINITE });
		recording.write_capture(capture);
		now = std::chrono::system_clock::now();
		long long duration_time = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

		if (duration_time > record_length)
		{
			std::cout << "Finished recording." << std::endl;
			break;
		}
	}

	device.stop_cameras();
	recording.flush();
	recording.close();
	device.close();

	return 0;
}

// 获取当前系统时间
inline std::string GetSystemTime()
{
	time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	struct tm ptm;
	localtime_s(&ptm, &tt);
	std::string year = std::to_string(ptm.tm_year + 1900);
	std::string month = std::to_string(ptm.tm_mon + 1);
	std::string day = std::to_string(ptm.tm_mday);
	std::string hour = std::to_string(ptm.tm_hour);
	std::string minute = std::to_string(ptm.tm_min);
	std::string second = std::to_string(ptm.tm_sec);
	std::string date = year + "_" + month + "_" + day + "_" + hour + "_" + minute + "_" + second;
	return date;
}