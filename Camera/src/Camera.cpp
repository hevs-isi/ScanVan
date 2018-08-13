//============================================================================
// Name        : Camera.cpp
// Author      : Marcelo Kaihara
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <unistd.h>
#include "Images.hpp"

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
		std::cout << curr_path << std::endl;
		std::string img_path = curr_path + "/" + "img_0_0.raw";
		std::cout << img_path << std::endl;
		std::string path {img_path};
		std::string ext = path.substr(path.find_last_of(".") + 1);
		std::cout << ext << std::endl;
		Images img1{path};
		img1.show();
		cv::waitKey(0);

	return 0;
}
