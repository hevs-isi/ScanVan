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

template<typename T>
std::string toString(const T &t) {
    std::ostringstream oss{};
    oss << t;
    return oss.str();
}

template<typename T>
T fromString( const std::string& s ) {
    std::istringstream stream( s );
    T t{};
    stream >> t;
    return t;
}

class Images {
private:
	std::vector<uint8_t> * p_img;
	size_t height = 3008;
	size_t width = 3008;
	size_t cameraIdx = 0;	// camera index
	time_t captureTime = 0;	// capture time
	double exposureTime = 0;// exposure time
	int64_t gain = 0;		// gain
	double balanceR = 0;	// white balance R
	double balanceG = 0;	// white balance G
	double balanceB = 0;	// white balance B
	int autoExpTime = 0;	// Auto Exposure Time
	int autoGain = 0; 		// Auto Gain

public:
	Images();
	Images(char * p);
	Images(size_t h, size_t w);
	Images(size_t h, size_t w, char * p);
	Images(std::string path);
	Images(const Images &img);
	Images(Images &&img);

	void setHeight(size_t h) {height = h;};
	void setWidth(size_t w) {width = w;};
	void setCameraIdx (size_t idx) { cameraIdx = idx; };
	void setCaptureTime (time_t ct) { captureTime = ct; };
	void setExposureTime (double et) { exposureTime = et; };
	void setGain (int64_t g) { gain = g; };
	void setBalanceR (double r) { balanceR = r; };
	void setBalanceG (double g) { balanceG = g; };
	void setBalanceB (double b) { balanceB = b; };
	void setAutoExpTime (int b) { autoExpTime = b; };
	void setAutoGain (int b) { autoGain = b; };

	size_t getHeight() { return height;};
	size_t getWidth() { return width;};
	size_t getCameraIdx() { return cameraIdx; };
	time_t getCaptureTime() { return captureTime; };
	double getExposureTime() { return exposureTime; };
	int64_t getGain() { return gain; };
	double getBalanceR () { return balanceR; };
	double getBalanceG () { return balanceG; };
	double getBalanceB () { return balanceB; };
	int getAutoExpTime() { return autoExpTime; };
	int getAutoGain() { return autoGain; };

	void getBuffer (char *p);
	void copyBuffer (char *p);

	void loadImage (std::string path);
	void saveImage (std::string path);
	void loadData (std::string path);
	void saveData (std::string path);
	void show ();
	void show (std::string name);

	Images & operator=(const Images &a);
	Images & operator=(Images &&a);

	virtual ~Images();

	friend std::ostream & operator <<(std::ostream & out, const Images &a) {
		out << "[";
		for (int i=0; i < 10; ++i) {
			out << static_cast<int>((*a.p_img)[i]) << " ";
		}
		out << "...]" << std::endl;
		out << "height: " << a.height << std::endl;
		out << "width: " << a.width << std::endl;
		out << "cameraIdx: " << a.cameraIdx << std::endl;
		out << "captureTime: " << a.captureTime << std::endl;
		out << "exposureTime: " << a.exposureTime << std::endl;
		out << "gain: " << a.gain << std::endl;
		out << "balanceR: " << a.balanceR << std::endl;
		out << "balanceG: " << a.balanceG << std::endl;
		out << "balanceB: " << a.balanceB << std::endl;
		out << "autoExpTime: " << a.autoExpTime << std::endl;
		out << "autoGain: " << a.autoGain << std::endl;

		return out;
	}
};

Images::Images(){
	p_img = new std::vector<uint8_t> {};
	p_img->reserve(height*width);
}

Images::Images(char * p){
	p_img = new std::vector<uint8_t> {};
	copyBuffer (p);
}

Images::Images(size_t h, size_t w) : height{h}, width{w} {
	p_img = new std::vector<uint8_t> {};
	p_img->reserve(height*width);
}

Images::Images(size_t h, size_t w, char * p) : height{h}, width{w} {
	p_img = new std::vector<uint8_t> {};
	copyBuffer(p);
}

Images::Images(std::string path) {
	p_img = new std::vector<uint8_t> {};
	loadImage(path);
}

Images::Images(const Images &img) {
	p_img = new std::vector<uint8_t> {*(img.p_img)};
}

Images::Images(Images &&img) {
	p_img = img.p_img;
	img.p_img = nullptr;
}

void Images::getBuffer(char *p) {
// copies the image to the buffer passed by the pointer p
	memcpy(p, p_img->data(), width*height);
}

void Images::copyBuffer(char *p) {
// copies the image passed by the pointer p into the buffer of the internal object
	p_img->assign(p,p+(height*width));
}

void Images::loadImage(std::string path) {
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


	std::string path_raw = path + ".raw";
	loadImage (path_raw);

	std::string path_data = path + ".txt";
	std::ifstream myFile(path_data);
	if (myFile.is_open()) {
		std::string line{};
		getline (myFile, line);
		getline (myFile, line);
		std::string token = line.substr(line.find_last_of(":") + 1);
		cameraIdx = std::stoi (token);
		std::cout << cameraIdx << std::endl;
		getline (myFile, line);
		std::string token2 = line.substr(line.find_last_of(":") + 2);
		captureTime = fromString<time_t> (token2);
		std::cout << "Capture Time: " << ctime(&captureTime);

		/*myFile << "Raw picture file: " << path_raw << "\n";
		myFile << "Camera Index: " << cameraIdx << "\n";
		//time_t my_time = captureTime;
		myFile << "Capture Time: " << ctime(&captureTime);
		myFile << "Exposure Time: " << exposureTime << "\n";
		myFile << "Gain: " << gain << "\n";
		myFile << "Balance Red  : " << balanceR << "\n";
		myFile << "Balance Green: " << balanceG << "\n";
		myFile << "Balance Blue : " << balanceB << "\n";
		myFile << "Auto Exposure Time Continuous: " << autoExpTime << "\n";
		myFile << "Auto Gain Continuous: " << autoGain << "\n";*/
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
		myFile << "Camera Index: " << cameraIdx << std::endl;
		//time_t my_time = captureTime;
		//myFile << "Capture Time: " << ctime(&captureTime);
		myFile << "Capture Time: " << toString<time_t>(captureTime) << std::endl;
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

Images & Images::operator=(const Images &a) {
	if (this != &a) {
		delete p_img;
		p_img = new std::vector<uint8_t> {*(a.p_img)};
	}
	return *this;
}

Images & Images::operator=(Images &&a) {
	if (this != &a) {
		delete p_img;
		p_img = a.p_img;
		a.p_img = nullptr;
	}
	return *this;
}

Images::~Images() {
	delete p_img;
}

#endif /* IMAGES_HPP_ */
