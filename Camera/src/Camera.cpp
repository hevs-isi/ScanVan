//============================================================================
// Name        : Camera.cpp
// Author      : Marcelo Kaihara
// Version     :
// Copyright   : 
// Description : Grabs images from the camera and writes the data into file.
//============================================================================

#include <iostream>
#include <unistd.h>
#include <chrono>
#include "Images.hpp"
#include <thread>

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


std::string GetCurrentWorkingDir( void ) {
// gets the current working directory
	std::array<char, FILENAME_MAX> buff { { } };
	if (getcwd(buff.data(), FILENAME_MAX) == nullptr) {
		throw(std::runtime_error("The directory could not be determined."));
	}
	std::string current_working_dir(buff.data());
	return current_working_dir;
}

int main() {

	std::string curr_path = GetCurrentWorkingDir();
	std::string img_path = curr_path + "/" + "img_0_0.raw";
	std::string img_path_out = curr_path + "/" + "img_0_1";
	Images img1 {img_path};
	img1.setCameraIdx(1);
	auto tnow = std::chrono::system_clock::now();
	time_t captureTime = std::chrono::system_clock::to_time_t(tnow);
	img1.setCaptureTime(captureTime);
	img1.setExposureTime(1.2);
	img1.setAutoExpTime(1);
	img1.setAutoGain(1);
	img1.setBalanceR(4.4);
	img1.setBalanceG(3.3);
	img1.setBalanceB(2.2);
	img1.setAutoGain(5);
	img1.setGain(5);

	img1.saveData(img_path_out);
	img1.show("Img1");
	cv::waitKey(0);
	Images img2 {};
	img2.loadData(img_path_out);

	return 0;
}
