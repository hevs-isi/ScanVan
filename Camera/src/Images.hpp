#ifndef IMAGES_HPP_
#define IMAGES_HPP_

#include <vector>
#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Images {
private:
	std::vector<uint8_t> * p_img;
	size_t height = 3008;
	size_t width = 3008;
public:
	Images();
	Images(char * p);
	Images(size_t h, size_t w);
	Images(size_t h, size_t w, char * p);
	Images(std::string path);
	void show ();
	virtual ~Images();
};

Images::Images(){
	p_img = new std::vector<uint8_t> {};
}

Images::Images(char * p){
	p_img = new std::vector<uint8_t> {};
	p_img->assign(p,p+(height*width));
}

Images::Images(size_t h, size_t w) : height{h}, width{w} {
	p_img = new std::vector<uint8_t> {};
}

Images::Images(size_t h, size_t w, char * p) : height{h}, width{w} {
	p_img = new std::vector<uint8_t> {};
	p_img->assign(p, p+(height*width));
}

Images::Images(std::string path) {
	p_img = new std::vector<uint8_t> {};
	std::string ext = path.substr(path.find_last_of(".") + 1);
	if (ext == "raw") {
		std::ifstream myFile(path, std::ios::in | std::ios::binary);
		if (myFile) {
			// get length of the file:
			myFile.seekg(0, myFile.end);
			int length = myFile.tellg();
			myFile.seekg(0, myFile.beg);
			p_img->reserve(length);
			char * buffer = new char[length];
			myFile.read(buffer, length);
			p_img->assign(buffer, buffer + length);
		}
	}
}

void Images::show() {

	cv::Mat openCvImageRG8 = cv::Mat(height, width, CV_8UC1, p_img->data());
	cv::Mat openCvImage;
	cv::cvtColor(openCvImageRG8, openCvImage, cv::COLOR_BayerRG2RGB);

	/// Display
	cv::namedWindow("Image", CV_WINDOW_NORMAL);
	cv::imshow("Image", openCvImage);
}

Images::~Images() {
	delete p_img;
}

#endif /* IMAGES_HPP_ */
