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
	std::string img_path = curr_path + "/" + "img_0_0.raw";
	Images img1 {img_path};
	char * newimg = new char [img1.getHeight() * img1.getWidth()] {};
	img1.getBuffer(newimg);
	Images img2 {};
	img2.copyBuffer(newimg);
	img2.show("Img2");
	img2.setCameraIdx(1);
	std::cout << img2 << std::endl;
	cv::waitKey(0);




	return 0;
}
