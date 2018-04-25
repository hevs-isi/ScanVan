//============================================================================
// Name        : raw2bmp.cpp
// Author      : Marcelo Kaihara
// Version     :
// Copyright   : 
// Description : Converts raw file into bmp
//============================================================================

#include <iostream>
#include <fstream>
#include <math.h>

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {

	if (argc < 2) {
		cerr << "Usage: " << argv[0] << " PATH_TO_RAW_IMAGE_FILE";
		return 1;
	}

	ostringstream s1;
	s1 << argv[1];
	string imageName(s1.str());

	ifstream myFile(imageName, ios::in | ios::binary);
	if (myFile) {
		// get length of the file:
		myFile.seekg(0, myFile.end);
		int length = myFile.tellg();
		myFile.seekg(0, myFile.beg);

		char * buffer = new char[length];
		myFile.read(buffer, length);

		if (myFile) {
			Mat openCvImageRG8;
			Mat openCvImage;

			int imageHeight = sqrt(length);
			if (imageHeight * imageHeight != length) {
				cerr << "Error in the dimension of the buffer size." << endl;
				return 1;
			}
			int imageWidth = imageHeight;

			openCvImageRG8 = cv::Mat(imageHeight, imageWidth, CV_8UC1, buffer);

			cvtColor(openCvImageRG8, openCvImage, COLOR_BayerRG2RGB);

			string str2="bmp";
			string imageBmpName = imageName;
			imageBmpName.replace(imageBmpName.end()-3, imageBmpName.end(),str2);

			try {
				imwrite(imageBmpName, openCvImage);
			}
			catch (runtime_error& ex) {
				cout << "Ëxception converting image to BMP format: " << ex.what() << endl;
				return 1;
			}
		} else
			cout << "Error: only " << myFile.gcount() << " could be read."
					<< endl;

		myFile.close();

	} else
		cout << "Error: file could not be opened."	<< endl;


	return 0;
}
