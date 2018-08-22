#ifndef IMAGES_CPP_
#define IMAGES_CPP_

#include "Images.hpp"

Images::Images(){
// Constructor, reserves memory for the image
	p_img = new std::vector<uint8_t> {};
	p_img->reserve(height*width);
}

Images::Images(char * p){
// Constructor where it receives a pointer to a buffer
// It copies the image to the object's buffer
	p_img = new std::vector<uint8_t> {};
	copyBuffer (p);
}

Images::Images(size_t h, size_t w) : height{h}, width{w} {
// Constructor that takes the height and the width as parameters
// It reserves memory for the image
	p_img = new std::vector<uint8_t> {};
	p_img->reserve(height*width);
}

Images::Images(size_t h, size_t w, char * p) : height{h}, width{w} {
// Constructor that takes the height and width and the pointer to the buffer
// It copies the image to the object's buffer
	p_img = new std::vector<uint8_t> {};
	copyBuffer(p);
}

Images::Images(std::string path) {
// Constructor that takes the path to the file where the raw image is stored
// It loads the image to the object's buffer
	p_img = new std::vector<uint8_t> {};
	loadImage(path);
}

Images::Images(const Images &img) {
// Copy constructor
	p_img = new std::vector<uint8_t> {*(img.p_img)};
	numImages = img.numImages;
	cameraIdx = img.cameraIdx;
	captureTime = img.captureTime;
	exposureTime = img.exposureTime;
	gain = img.gain;
	balanceR = img.balanceR;
	balanceG = img.balanceG;
	balanceB = img.balanceB;
	autoExpTime = img.autoExpTime;
	autoGain = img.autoGain;
}

Images::Images(Images &&img) {
// Move constructor
	p_img = img.p_img;
	img.p_img = nullptr;

	cameraIdx = img.cameraIdx;
	captureTime = img.captureTime;
	exposureTime = img.exposureTime;
	gain = img.gain;
	balanceR = img.balanceR;
	balanceG = img.balanceG;
	balanceB = img.balanceB;
	autoExpTime = img.autoExpTime;
	autoGain = img.autoGain;
}

void Images::getBuffer(char *p) {
// copies the image stored in the object's buffer to the buffer passed by the pointer p
	memcpy(p, p_img->data(), width*height);
}

void Images::copyBuffer(char *p) {
// copies the image passed by the pointer p into the object's buffer
	p_img->assign(p,p+(height*width));
}

void Images::loadImage(std::string path) {
// Loads the images from the passed path.
// The file needs to be a .raw file.
	std::string ext = path.substr(path.find_last_of(".") + 1);
	if (ext == "raw") {
		std::ifstream myFile(path, std::ios::in | std::ios::binary);
		if (myFile) {
			// get length of the file:
			myFile.seekg(0, myFile.end);
			int length = myFile.tellg();
			myFile.seekg(0, myFile.beg);
			p_img->reserve(length);

			char * buffer = new char[length] { };
			myFile.read(buffer, length);
			p_img->assign(buffer, buffer + length);
			myFile.close();
		} else {
			throw std::runtime_error("Image file extension not recognized.");
		}
	} else {
		throw std::runtime_error("Image file extension not recognized.");
	}
}

void Images::saveImage(std::string path) {
// It saves the object's image to file
// If the path contains the extension .raw, it saves the raw data
// If the path contains the extension .bmp, it saves in bmp format
	std::string ext = path.substr(path.find_last_of(".") + 1);

	// Save the raw image into file
	if (ext == "raw") {
		std::ofstream myFile(path, std::ios::out | std::ios::binary);
		if (myFile.is_open()) {
			myFile.write(reinterpret_cast<char*>(p_img->data()), height * width);
			myFile.close();
		} else {
			throw std::runtime_error("Error to write the image file");
		}
	} else if (ext == "bmp") {
		cv::Mat openCvImageRG8;
		cv::Mat openCvImage;
		openCvImageRG8 = cv::Mat(height, width, CV_8UC1, p_img->data());
		cv::cvtColor(openCvImageRG8, openCvImage, cv::COLOR_BayerRG2RGB);
		try {
			imwrite(path, openCvImage);
		}
		catch (std::runtime_error& ex) {
			std::cerr << "Error writing the bmp file: " << ex.what() << std::endl;
			throw ex;
		}

	} else {
		throw std::runtime_error("File extension not recognized when trying to save the image.");
	}
}

void Images::loadData(std::string path) {
// It loads the image from the .raw file and the camera parameters from the .txt file.
// The provided path is the base name and the extension will be appended.

	std::string path_raw = path + ".raw";
	loadImage (path_raw);

	std::string path_data = path + ".txt";
	std::ifstream myFile(path_data);
	if (myFile.is_open()) {
		std::stringstream ss{};
		std::string line{};
		getline (myFile, line);

		getline(myFile, line);
		std::string token = line.substr(line.find_last_of(":") + 1);
		ss << token;
		ss >> numImages;
		std::cout << "Image Number: " << numImages << std::endl;


		getline (myFile, line);
		token = line.substr(line.find_last_of(":") + 1);
		ss.str(std::string());
		ss.clear();
		ss << token;
		ss >> cameraIdx;
		std::cout << "Camera Index: " << cameraIdx << std::endl;

		getline (myFile, line);
		token = line.substr(line.find_first_of(":") + 1);
		captureTime = convertStringToTime (token);
		std::cout << "Capture Time: " << ctime(&captureTime);

		getline (myFile, line);
		token = line.substr(line.find_last_of(":") + 1);
		ss.str(std::string());
		ss.clear();
		ss << token;
		ss >> exposureTime;
		std::cout << "Exposure Time: " << exposureTime << std::endl;

		getline (myFile, line);
		token = line.substr(line.find_last_of(":") + 1);
		ss.str(std::string());
		ss.clear();
		ss << token;
		ss >> gain;
		std::cout << "Gain: " << gain << std::endl;

		getline (myFile, line);
		token = line.substr(line.find_last_of(":") + 1);
		ss.str(std::string());
		ss.clear();
		ss << token;
		ss >> balanceR;
		std::cout << "Balance Red: " << balanceR << std::endl;

		getline (myFile, line);
		token = line.substr(line.find_last_of(":") + 1);
		ss.str(std::string());
		ss.clear();
		ss << token;
		ss >> balanceG;
		std::cout << "Balance Green: " << balanceG << std::endl;

		getline (myFile, line);
		token = line.substr(line.find_last_of(":") + 1);
		ss.str(std::string());
		ss.clear();
		ss << token;
		ss >> balanceB;
		std::cout << "Balance Blue: " << balanceB << std::endl;

		getline(myFile, line);
		token = line.substr(line.find_last_of(":") + 1);
		ss.str(std::string());
		ss.clear();
		ss << token;
		ss >> autoExpTime;
		std::cout << "Auto Exposure Time Continuous: " << autoExpTime << std::endl;

		getline(myFile, line);
		token = line.substr(line.find_last_of(":") + 1);
		ss.str(std::string());
		ss.clear();
		ss << token;
		ss >> autoGain;
		std::cout << "Auto Gain Continuous: " << autoGain << std::endl;

		myFile.close();
	} else {
		throw std::runtime_error("Could not open the file to load camera data");
	}
}


void Images::saveData(std::string path) {
// Saves the raw image and the camera data to file
// Here path is the name of the image file without extension
// The function will automatically add the .raw for the raw data image and .txt for the camera
// configuration.
	std::string ext = path.substr(path.find_last_of(".") + 1);

	std::string path_raw = path + ".raw";
	saveImage (path_raw);
	//std::string path_bmp = path + ".bmp";
	//saveImage (path_bmp);
	std::string path_data = path + ".txt";
	std::ofstream myFile(path_data);
	if (myFile.is_open()) {
		myFile << "Raw picture file: " << path_raw << std::endl;
		myFile << "Image number: " << numImages << std::endl;
		myFile << "Camera Index: " << cameraIdx << std::endl;
		myFile << "Capture Time: " << convertTimeToString(captureTime) << std::endl;
		myFile << "Exposure Time: " << exposureTime << std::endl;
		myFile << "Gain: " << gain << std::endl;
		myFile << "Balance Red  : " << balanceR << std::endl;
		myFile << "Balance Green: " << balanceG << std::endl;
		myFile << "Balance Blue : " << balanceB << std::endl;
		myFile << "Auto Exposure Time Continuous: " << autoExpTime << std::endl;
		myFile << "Auto Gain Continuous: " << autoGain << std::endl;
		myFile.close();
	} else {
		throw std::runtime_error ("Could not open the file to save camera data");
	}
}

void Images::show() {
// It shows the image in an opencv window with the title "Image"
	cv::Mat openCvImageRG8 = cv::Mat(height, width, CV_8UC1, p_img->data());
	cv::Mat openCvImage;
	cv::cvtColor(openCvImageRG8, openCvImage, cv::COLOR_BayerRG2RGB);

	/// Display
	cv::namedWindow("Image", CV_WINDOW_NORMAL);
	cv::imshow("Image", openCvImage);
}

void Images::show (std::string name) {
// shows the image in an opencv window with the name provided as parameter
	cv::Mat openCvImageRG8 = cv::Mat(height, width, CV_8UC1, p_img->data());
	cv::Mat openCvImage;
	cv::cvtColor(openCvImageRG8, openCvImage, cv::COLOR_BayerRG2RGB);

	/// Display
	cv::namedWindow(name, CV_WINDOW_NORMAL);
	cv::imshow(name, openCvImage);
}

std::string Images::convertTimeToString (time_t t) {
// It converts the time t into a string
	struct tm *theTime{};
	theTime = localtime(&t);
	std::stringstream ss{};

	ss << std::setw(4) << std::setfill('0') << (1900 + theTime->tm_year) <<  "/";
	ss << std::setw(2) << std::setfill('0') << theTime->tm_mon + 1 << "/";
	ss << std::setw(2) << std::setfill('0') << theTime->tm_mday << " - ";
	ss << std::setw(2) << std::setfill('0') << theTime->tm_hour << ":";
	ss << std::setw(2) << std::setfill('0') << theTime->tm_min << ":";
	ss << std::setw(2) << std::setfill('0') << theTime->tm_sec << " (DST: ";
	ss << std::setw(2) << std::setfill('0') << theTime->tm_isdst << ")";

	return ss.str();
}

time_t Images::convertStringToTime (std::string str) {
// It converts from a string into a time_t variable
	struct tm st{};
	time_t t{};
	std::stringstream ss{};

	int num{};
	char c{};
	ss << str;

	ss >> num;
	st.tm_year = num - 1900;
	ss >> c;
	ss >> num;
	st.tm_mon = num - 1;
	ss >> c;
	ss >> num;
	st.tm_mday = num;
	ss >> c;
	ss >> num;
	st.tm_hour = num;
	ss >> c;
	ss >> num;
	st.tm_min = num;
	ss >> c;
	ss >> num;
	st.tm_sec = num;
	ss >> c;
	ss >> c;
	ss >> c;
	ss >> c;
	ss >> c;
	ss >> num;
	st.tm_isdst = num;

	t = mktime(&st);

	return t;
}

Images & Images::operator=(const Images &a) {
	if (this != &a) {
		delete p_img;
		p_img = new std::vector<uint8_t> {*(a.p_img)};
		numImages = a.numImages;
		cameraIdx = a.cameraIdx;
		captureTime = a.captureTime;
		exposureTime = a.exposureTime;
		gain = a.gain;
		balanceR = a.balanceR;
		balanceG = a.balanceG;
		balanceB = a.balanceB;
		autoExpTime = a.autoExpTime;
		autoGain = a.autoGain;
	}
	return *this;
}

Images & Images::operator=(Images &&a) {
	if (this != &a) {
		delete p_img;
		p_img = a.p_img;
		a.p_img = nullptr;
		numImages = a.numImages;
		cameraIdx = a.cameraIdx;
		captureTime = a.captureTime;
		exposureTime = a.exposureTime;
		gain = a.gain;
		balanceR = a.balanceR;
		balanceG = a.balanceG;
		balanceB = a.balanceB;
		autoExpTime = a.autoExpTime;
		autoGain = a.autoGain;
	}
	return *this;
}

Images::~Images() {
	delete p_img;
}

#endif /* IMAGES_CPP_ */
