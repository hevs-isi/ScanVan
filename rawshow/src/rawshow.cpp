//============================================================================
// Name        : rawshow.cpp
// Author      : Marcelo Kaihara
// Version     :
// Copyright   : 
// Description : Displays the raw image in RG8
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
		cerr << "Usage: " << argv[0] << " PATH_TO_RAW_IMAGE_FILE" << endl;
		return 1;
	}

	std::ostringstream s1;
	s1 << argv[1];
	std::string imageName(s1.str());

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

			// Specify the name of the window to show
			String windowTitle = argv[1];

			// Create an OpenCV display window.
			namedWindow(windowTitle, CV_WINDOW_NORMAL);	// other options: // CV_AUTOSIZE, CV_FREERATIO

			// Display the current image in the OpenCV display window.
			imshow(windowTitle, openCvImage);

			// Define a timeout for customer's input in ms.
			// '0' means indefinite, i.e. the next image will be displayed after closing the window.
			// '1' means live stream
			waitKey(0);

		} else
			cout << "Error: only " << myFile.gcount() << " could be read."
					<< endl;

		myFile.close();

	} else
		cout << "Error: file could not be opened."	<< endl;


	return 0;
}
