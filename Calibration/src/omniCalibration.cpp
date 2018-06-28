#include "omnidir.cpp"
#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <time.h>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;

const char * usage =
		"\n example command line for omnidirectional camera calibration.\n"
				"    omni_calibration -w 6 -h 9 -sw 80 -sh 80 imagelist.xml \n"
				" \n"
				" the file imagelist.xml is generated by imagelist_creator as\n"
				"imagelist_creator imagelist.xml *.*";

static void help() {
	printf(
			"\n This is a sample for omnidirectional camera calibration.\n"
					"Usage: omni_calibration\n"
					"    -w <board_width>    # the number of inner corners per one of board dimension\n"
					"    -h <board_height>    # the number of inner corners per another board dimension\n"
					"    [-sw <square_width>] # the width of square in some user-defined units (1 by default)\n"
					"    [-sh <square_height>] # the height of square in some user-defined units (1 by default)\n"
					"    [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
					"    [-fs <fix_skew>] # fix skew\n"
					"    [-fp ] # fix the principal point at the center\n"
					"    input_data # input data - text file with a list of the images of the board, which is generated by imagelist_creator");
	printf("\n %s", usage);
}

static void calcChessboardCorners(Size boardSize, double square_width,
		double square_height, Mat& corners) {
	// corners has type of CV_64FC3
	corners.release();
	int n = boardSize.width * boardSize.height;
	corners.create(n, 1, CV_64FC3);
	Vec3d *ptr = corners.ptr<Vec3d>();
	for (int i = 0; i < boardSize.height; ++i) {
		for (int j = 0; j < boardSize.width; ++j) {
			ptr[i * boardSize.width + j] = Vec3d(double(j * square_width),
					double(i * square_height), 0.0);
		}
	}
}

static bool detecChessboardCorners(const vector<string>& list,
		vector<string>& list_detected, vector<Mat>& imagePoints, Size boardSize,
		Size& imageSize) {
	imagePoints.resize(0);
	list_detected.resize(0);
	int n_img = (int) list.size();
	Mat img; //,imgtemp;
	for (int i = 0; i < n_img; ++i) {
		Mat points;
		cout << list[i] << endl;
		img = imread(list[i], IMREAD_GRAYSCALE);
		//imgtemp = imread(list[i], IMREAD_COLOR|IMREAD_ANYDEPTH);
		//cvtColor(imgtemp, img, CV_BGR2GRAY);
		bool found = findChessboardCorners(img, boardSize, points);
		if (found) {
			if (points.type() != CV_64FC2)
				points.convertTo(points, CV_64FC2);
			imagePoints.push_back(points);
			list_detected.push_back(list[i]);
		}
	}
	if (!img.empty())
		imageSize = img.size();
	if (imagePoints.size() < 3)
		return false;
	else
		return true;
}

static bool readStringList(const string& filename, vector<string>& l) {
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string) *it);
	return true;
}

static void saveCameraParams(const string & filename, int flags,
		const Mat& cameraMatrix, const Mat& distCoeffs, const double xi,
		const vector<Vec3d>& rvecs, const vector<Vec3d>& tvecs,
		vector<string> detec_list, const Mat& idx, const double rms,
		const vector<Mat>& imagePoints, Size imageSize) {
	FileStorage fs(filename, FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty())
		fs << "nFrames" << (int) rvecs.size();

	if (flags != 0) {
		sprintf(buf, "flags: %s%s%s%s%s%s%s%s%s",
				flags & omnidir::CALIB_USE_GUESS ? "+use_intrinsic_guess" : "",
				flags & omnidir::CALIB_FIX_SKEW ? "+fix_skew" : "",
				flags & omnidir::CALIB_FIX_K1 ? "+fix_k1" : "",
				flags & omnidir::CALIB_FIX_K2 ? "+fix_k2" : "",
				flags & omnidir::CALIB_FIX_P1 ? "+fix_p1" : "",
				flags & omnidir::CALIB_FIX_P2 ? "+fix_p2" : "",
				flags & omnidir::CALIB_FIX_XI ? "+fix_xi" : "",
				flags & omnidir::CALIB_FIX_GAMMA ? "+fix_gamma" : "",
				flags & omnidir::CALIB_FIX_CENTER ? "+fix_center" : "");
		//cvWriteComment( *fs, buf, 0 );
	}

	fs << "flags" << flags;

	fs << "imageSize" << imageSize;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs << "xi" << xi;

	//cvWriteComment( *fs, "names of images that are acturally used in calibration", 0 );
	fs << "used_imgs" << "[";
	for (int i = 0; i < (int) idx.total(); ++i) {
		fs << detec_list[(int) idx.at<int>(i)];
	}
	fs << "]";

	if (!rvecs.empty() && !tvecs.empty()) {
		Mat rvec_tvec((int) rvecs.size(), 6, CV_64F);
		for (int i = 0; i < (int) rvecs.size(); ++i) {
			Mat(rvecs[i]).reshape(1, 1).copyTo(rvec_tvec(Rect(0, i, 3, 1)));
			Mat(tvecs[i]).reshape(1, 1).copyTo(rvec_tvec(Rect(3, i, 3, 1)));
		}
		//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
		fs << "extrinsic_parameters" << rvec_tvec;
	}

	fs << "rms" << rms;

	if (!imagePoints.empty()) {
		Mat imageMat((int) imagePoints.size(), (int) imagePoints[0].total(),
		CV_64FC2);
		for (int i = 0; i < (int) imagePoints.size(); ++i) {
			Mat r = imageMat.row(i).reshape(2, imageMat.cols);
			Mat imagei(imagePoints[i]);
			imagei.copyTo(r);
		}
		fs << "image_points" << imageMat;
	}
}


int main(int argc, char** argv) {
	Size boardSize, imageSize, imageSizeUndistort;
	int flags = 0;
	double square_width = 0.0, square_height = 0.0;
	const char* outputFilename = "out_camera_omni.xml";
	const char* inputFilename = 0;
	vector<Mat> objectPoints;
	vector<Mat> imagePoints;
	double pi=3.141592653589793;

	if (argc < 2) {
		help();
		return 1;
	}

	for (int i = 1; i < argc; i++) {
		const char* s = argv[i];
		if (strcmp(s, "-w") == 0) {
			if (sscanf(argv[++i], "%u", &boardSize.width) != 1
					|| boardSize.width <= 0)
				return fprintf( stderr, "Invalid board width\n"), -1;
		} else if (strcmp(s, "-h") == 0) {
			if (sscanf(argv[++i], "%u", &boardSize.height) != 1
					|| boardSize.height <= 0)
				return fprintf( stderr, "Invalid board height\n"), -1;
		} else if (strcmp(s, "-sw") == 0) {
			if (sscanf(argv[++i], "%lf", &square_width) != 1
					|| square_width <= 0)
				return fprintf(stderr, "Invalid square width\n"), -1;
		} else if (strcmp(s, "-sh") == 0) {
			if (sscanf(argv[++i], "%lf", &square_height) != 1
					|| square_height <= 0)
				return fprintf(stderr, "Invalid square height\n"), -1;
		} else if (strcmp(s, "-o") == 0) {
			outputFilename = argv[++i];
		} else if (strcmp(s, "-fs") == 0) {
			flags |= omnidir::CALIB_FIX_SKEW;
		} else if (strcmp(s, "-fp") == 0) {
			flags |= omnidir::CALIB_FIX_CENTER;
		} else if (s[0] != '-') {
			inputFilename = s;
		} else {
			return fprintf( stderr, "Unknown option %s\n", s), -1;
		}
	}

	// get image name list
	vector<string> image_list, detec_list;
	if (!readStringList(inputFilename, image_list))
		return fprintf( stderr, "Failed to read image list\n"), -1;

	// find corners in images
	// some images may be failed in automatic corner detection, passed cases are in detec_list
	if (!detecChessboardCorners(image_list, detec_list, imagePoints, boardSize,
			imageSize))
		return fprintf(stderr, "Not enough corner detected images\n"), -1;

	// calculate object coordinates
	Mat object;
	calcChessboardCorners(boardSize, square_width, square_height, object);
	for (int i = 0; i < (int) detec_list.size(); ++i)
		objectPoints.push_back(object);


	// run calibration, some images are discarded in calibration process because they are failed
	// in initialization. Retained image indexes are in idx variable.
	Mat K, D, xi, idx;
	vector<Vec3d> rvecs, tvecs;
	double _xi, rms;
	TermCriteria criteria(3, 200, 1e-8);
	rms = omnidir::calibrate(objectPoints, imagePoints, imageSize, K, xi, D,
			rvecs, tvecs, flags, criteria, idx);
	_xi = xi.at<double>(0);
	saveCameraParams(outputFilename, flags, K, D, _xi, rvecs, tvecs, detec_list,
			idx, rms, imagePoints, imageSize);

	int n_img = (int) image_list.size();

	Mat undistorted;	// destination image
	Mat R = Mat::eye(3, 3, CV_64F);	// Additional transformation on the sphere
	//R.at<double>(0, 0) = 0.5;
	//R.at<double>(1, 1) = 0.5;

	imageSizeUndistort.width = imageSize.width;  // Approximately the diameter of the mirror on the sensor image
	imageSizeUndistort.height = imageSize.width; // The mirror is circle, set to the width of the mirror

	// Transformation matrix from the spherical coordinates to destination image
	Matx33f Knew(imageSizeUndistort.width / pi, 0, imageSizeUndistort.width / 2, 0, imageSizeUndistort.height / pi, 0,
					0, 0, 1);


	cv::Mat map1, map2;
	omnidir::initUndistortRectifyMap(K, D, _xi, R, Knew, imageSizeUndistort, CV_16SC2, map1, map2, cv::omnidir::RECTIFY_LONGLATI);
	//omnidir::initUndistortRectifyMap(K, D, _xi, R, cv::noArray(), imageSizeUndistort, CV_16SC2, map1, map2, cv::omnidir::RECTIFY_PERSPECTIVE);



	long accum(0); // variable to measure the running time


	for (int i = 0; i < n_img; ++i) {
		// load the images and store them in a vector
		//Mat distorted = imread(image_list[i], IMREAD_COLOR|IMREAD_ANYDEPTH);
		Mat distorted = imread(image_list[i], IMREAD_COLOR);

		cout << image_list[i] << endl << flush;

		high_resolution_clock::time_point t1 = high_resolution_clock::now();

		// main remapping function that undistort the images
		cv::remap(distorted, undistorted, map1, map2, INTER_CUBIC, BORDER_CONSTANT);

		high_resolution_clock::time_point t2 = high_resolution_clock::now();

		auto duration = duration_cast<microseconds>(t2-t1).count();
		cout << duration << " microseconds" << endl << flush;
		accum += duration;

		//save images into file
		imwrite(string("img") + std::to_string(i) + ".bmp", undistorted);
	}
	cout << "Average duration = " << accum/n_img << " microseconds" << endl << flush;



	/* Transform the images from a directory */
	for (int i = 0; i <= 161; ++i) {
		// load the images and store them in a vector
		//Mat distorted = imread(image_list[i], IMREAD_COLOR|IMREAD_ANYDEPTH);

		ostringstream s1;
		// Create image name files with ascending grabbed image numbers.
		s1 << "/home/scanvandev/ScanVan/Calibration/img/trajectoryA_fixed" << "/img_0_" << i << ".bmp";
		string imageFileName(s1.str());

		Mat distorted = imread(imageFileName, IMREAD_COLOR);

		cout << imageFileName << endl << flush;

		// main remapping function that undistort the images
		cv::remap(distorted, undistorted, map1, map2, INTER_CUBIC, BORDER_CONSTANT);

		ostringstream s2;
		s2 << "/home/scanvandev/ScanVan/Calibration/img/trajectoryA_fixed/equirectangular" << "/img_equi_0_" << i << ".bmp";
		string outputFileName(s2.str());

		//save images into file
		imwrite(outputFileName, undistorted);
	}

	for (int i = 0; i <= 188; ++i) {
			// load the images and store them in a vector
			//Mat distorted = imread(image_list[i], IMREAD_COLOR|IMREAD_ANYDEPTH);

			ostringstream s1;
			// Create image name files with ascending grabbed image numbers.
			s1 << "/home/scanvandev/ScanVan/Calibration/img/trajectoryB_auto" << "/img_0_" << i << ".bmp";
			string imageFileName(s1.str());

			Mat distorted = imread(imageFileName, IMREAD_COLOR);

			cout << imageFileName << endl << flush;

			// main remapping function that undistort the images
			cv::remap(distorted, undistorted, map1, map2, INTER_CUBIC, BORDER_CONSTANT);

			ostringstream s2;
			s2 << "/home/scanvandev/ScanVan/Calibration/img/trajectoryB_auto/equirectangular" << "/img_equi_0_" << i << ".bmp";
			string outputFileName(s2.str());

			//save images into file
			imwrite(outputFileName, undistorted);
	}

	for (int i = 0; i <= 183; ++i) {
				// load the images and store them in a vector
				//Mat distorted = imread(image_list[i], IMREAD_COLOR|IMREAD_ANYDEPTH);

				ostringstream s1;
				// Create image name files with ascending grabbed image numbers.
				s1 << "/home/scanvandev/ScanVan/Calibration/img/trajectoryC_fixed" << "/img_0_" << i << ".bmp";
				string imageFileName(s1.str());

				Mat distorted = imread(imageFileName, IMREAD_COLOR);

				cout << imageFileName << endl << flush;

				// main remapping function that undistort the images
				cv::remap(distorted, undistorted, map1, map2, INTER_CUBIC, BORDER_CONSTANT);

				ostringstream s2;
				s2 << "/home/scanvandev/ScanVan/Calibration/img/trajectoryC_fixed/equirectangular" << "/img_equi_0_" << i << ".bmp";
				string outputFileName(s2.str());

				//save images into file
				imwrite(outputFileName, undistorted);
	}

	for (int i = 0; i <= 17; ++i) {
					// load the images and store them in a vector
					//Mat distorted = imread(image_list[i], IMREAD_COLOR|IMREAD_ANYDEPTH);

					ostringstream s1;
					// Create image name files with ascending grabbed image numbers.
					s1 << "/home/scanvandev/ScanVan/Calibration/img/trajectoryD_calib" << "/img_0_" << i << ".bmp";
					string imageFileName(s1.str());

					Mat distorted = imread(imageFileName, IMREAD_COLOR);

					cout << imageFileName << endl << flush;

					// main remapping function that undistort the images
					cv::remap(distorted, undistorted, map1, map2, INTER_CUBIC, BORDER_CONSTANT);

					ostringstream s2;
					s2 << "/home/scanvandev/ScanVan/Calibration/img/trajectoryD_calib/equirectangular" << "/img_equi_0_" << i << ".bmp";
					string outputFileName(s2.str());

					//save images into file
					imwrite(outputFileName, undistorted);
	}



	/*namedWindow("Rectified", WINDOW_NORMAL);
	for (int i = 0; i < n_img; ++i) {
		//Mat distorted = imread(image_list[i], IMREAD_GRAYSCALE);
		Mat distorted = imread(image_list[i], IMREAD_COLOR|IMREAD_ANYDEPTH);
		imshow("Rectified", distorted);
		imageSizeUndistort.width / 2, resizeWindow("Rectified",
				imageSize.width / 2, imageSize.height / 2);
		waitKey(0);

		cv::remap(distorted, undistorted, map1, map2, INTER_CUBIC, BORDER_CONSTANT);

		imshow("Rectified", undistorted);
		resizeWindow("Rectified", imageSizeUndistort.width,	imageSizeUndistort.height);
		waitKey(0);
	}
*/


}

