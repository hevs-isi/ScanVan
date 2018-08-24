#ifndef IMAGES_HPP_
#define IMAGES_HPP_

#include <vector>
#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <sstream>

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ScanVan {

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
	long int numImages = 0;
	std::string serialNumber{};

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
	void setSerialNumber (std::string sn) { serialNumber = sn; };

	size_t getHeight() { return height;};
	size_t getWidth() { return width;};
	size_t getCameraIdx() { return cameraIdx; };
	time_t getCaptureTime() { return captureTime; };
	double getExposureTime() { return exposureTime; };
	int64_t getGain() { return gain; };
	double getBalanceR () { return balanceR; };
	double getBalanceG () { return balanceG; };
	double getBalanceB () { return balanceB; };
	std::string getSerialNumber() { return serialNumber; };
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

	std::string convertTimeToString (time_t t);
	time_t convertStringToTime (std::string str);

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

} /* namespace ScanVan */

#endif /* IMAGES_HPP_ */
