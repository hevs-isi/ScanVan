//============================================================================
// Name        : PoseEstimation.cpp
// Author      : Marcelo Kaihara
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <vector>

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <limits>

using namespace std;
// Namespace for using OpenCV objects.
using namespace cv;

typedef struct Point {
	double x;
	double y;
	double z;
} Point_t;

typedef numeric_limits<double> dbl;

// To count the number of operations
int numMul(0), numMulAdd(0), numAdd(0),numDeterm(0), numDiv(0), numSVD(0), numMatInv(0), numSqrt(0);

void printMatrix(Mat M) {
	cout << "Type of matrix: " << M.type() << endl;
	// Don't print empty matrices
	if (M.empty()) {
		cout << "---" << endl;
		return;
	}
	// loop through columns and rows of the matrix
	for (int i = 0; i < M.rows; i++) {
		for (int j = 0; j < M.cols; j++) {
			cout << M.at<double>(i, j) << "  ";
		}
		cout << endl;
	}
}

int loadVector (string inputFileName, vector<Point_t> &p)
{
	ifstream inputFile;
	string str(""), str1(""), str2(""), str3(""), str4("");
	string token;
	Point_t point;
	int linenum = 0;

	inputFile.open (inputFileName);
	if (inputFile.is_open()) {

		while (!inputFile.eof()) {
			getline(inputFile, str);
			if (str != "") {
				if (linenum == 0) {
					str1 = str.substr(str.find("[") + 1, str.length());
				} else {
					str1 = str;
				}
				str2 = str1.substr(str1.find("[") + 1, str1.length());
				token = str2.substr(0, str2.find(",") - 1);
				point.x = stod(token, NULL);
				str3 = str2.substr(str2.find(",") + 1, str2.length());
				token = str3.substr(0, str3.find(",") - 1);
				point.y = stod(token, NULL);
				str4 = str3.substr(str3.find(",") + 1, str3.length());
				token = str4.substr(0, str4.find("]") - 1);
				point.z = stod(token, NULL);

				p.push_back(point);
				linenum++;
			}
		}
		inputFile.close();
	} else {
		cout << "Error: Unable to open the file \"" << inputFileName << "\"";
		return 1;
	}

	return 0;
}

int loadVectors (vector<Point_t> &p3d_1, vector<Point_t> &p3d_2, vector<Point_t> &p3d_3)
{

	string inputFileName = "/home/scanvandev/ScanVan/PoseEstimation/C++/p3d_1.txt";
	if (loadVector(inputFileName, p3d_1))
		return 1;
	inputFileName = "/home/scanvandev/ScanVan/PoseEstimation/C++/p3d_2.txt";
	if (loadVector(inputFileName, p3d_2))
		return 1;
	inputFileName = "/home/scanvandev/ScanVan/PoseEstimation/C++/p3d_3.txt";
	if (loadVector(inputFileName, p3d_3))
		return 1;

	return 0;
}

int writeVector (string outputFileName, vector<Point_t> &sv_scene) {

	ofstream outputFile;

	outputFile.open (outputFileName);
	outputFile.precision(dbl::max_digits10);
	if (outputFile.is_open()) {
		outputFile << "[";
		for (size_t i=0; i<sv_scene.size()-1; i++) {
			outputFile << "[" << fixed << sv_scene[i].x << ", " <<
					fixed << sv_scene[i].y << ", " <<
					fixed << sv_scene[i].z << "];\n";
		}
		outputFile << "[" << fixed << sv_scene[sv_scene.size()-1].x << ", " <<
							fixed << sv_scene[sv_scene.size()-1].y << ", " <<
							fixed << sv_scene[sv_scene.size()-1].z << "]];\n";
		outputFile.close();
	} else {
		cout << "Error: Unable to open the file \"" << outputFileName << "\"";
		return 1;
	}

	return 0;
}

void svd_rotation (const Mat &v, const Mat &u, Mat &vmu)
{
	//int numDeterm(0),numMulAdd(0), numMul(0);

	Mat m = Mat::eye(3, 3, CV_64FC1);
	m.at<double>(2, 2) = determinant (v * u);
	numMulAdd += 2 * 3;
	numMul += 3;
	numDeterm ++;

	vmu = v * m * u;
	numMulAdd += 3 * 3 + 3 * 3;

	/*cout << "svd_rotation:" << endl;
	cout << " #numMul: " << numMul << endl;
	cout << " #numMulAdd: " << numMulAdd << endl;
	cout << " #numDeterm: " << numDeterm << endl;
*/
}

void estimation_rot_trans (const vector<Point_t> &p3d_1, const vector<Point_t> &p3d_2, const vector<Point_t> &p3d_3,
						   const vector<double> &sv_u, const vector<double> &sv_v, const vector<double> sv_w,
						   Mat &sv_r_12, Mat &sv_r_23, Mat &sv_r_31, Mat &sv_t_12, Mat &sv_t_23, Mat &sv_t_31)
{

	//int numMul(0), numMulAdd(0), numDiv(0), numAdd(0), numSVD(0);
	vector<Point_t> p3d_1_exp, p3d_2_exp, p3d_3_exp;
	Point_t p;
	int length = p3d_1.size();

	// Scale the points
	for (int i=0; i<length; ++i) {
		p.x = p3d_1[i].x * sv_u[i];
		p.y = p3d_1[i].y * sv_u[i];
		p.z = p3d_1[i].z * sv_u[i];
		p3d_1_exp.push_back(p);
		numMul += 3;
	}
	for (int i=0; i<length; ++i) {
		p.x = p3d_2[i].x * sv_v[i];
		p.y = p3d_2[i].y * sv_v[i];
		p.z = p3d_2[i].z * sv_v[i];
		p3d_2_exp.push_back(p);
		numMul += 3;
	}
	for (int i=0; i<length; ++i) {
		p.x = p3d_3[i].x * sv_w[i];
		p.y = p3d_3[i].y * sv_w[i];
		p.z = p3d_3[i].z * sv_w[i];
		p3d_3_exp.push_back(p);
		numMul += 3;
	}

	// Compute centroids of the point sets
	Point_t sv_cent_1 {0,0,0};
	Point_t sv_cent_2 {0,0,0};
	Point_t sv_cent_3 {0,0,0};
	for (int i=0; i<length; ++i) {
		sv_cent_1.x += p3d_1_exp[i].x;
		sv_cent_1.y += p3d_1_exp[i].y;
		sv_cent_1.z += p3d_1_exp[i].z;
		numAdd += 3;
	}
	sv_cent_1.x /= length;
	sv_cent_1.y /= length;
	sv_cent_1.z /= length;
	numDiv += 3;

	for (int i = 0; i < length; ++i) {
		sv_cent_2.x += p3d_2_exp[i].x;
		sv_cent_2.y += p3d_2_exp[i].y;
		sv_cent_2.z += p3d_2_exp[i].z;
		numAdd += 3;
	}
	sv_cent_2.x /= length;
	sv_cent_2.y /= length;
	sv_cent_2.z /= length;
	numDiv += 3;

	for (int i = 0; i < length; ++i) {
		sv_cent_3.x += p3d_3_exp[i].x;
		sv_cent_3.y += p3d_3_exp[i].y;
		sv_cent_3.z += p3d_3_exp[i].z;
		numAdd += 3;
	}
	sv_cent_3.x /= length;
	sv_cent_3.y /= length;
	sv_cent_3.z /= length;
	numDiv += 3;

	// Compute the centered vectors
	Mat sv_diff_1 (length, 3, CV_64FC1);
	Mat sv_diff_2 (length, 3, CV_64FC1);
	Mat sv_diff_3 (length, 3, CV_64FC1);

	for (int i = 0; i < length; ++i) {
		sv_diff_1.at<double>(i, 0) = p3d_1_exp[i].x - sv_cent_1.x;
		sv_diff_1.at<double>(i, 1) = p3d_1_exp[i].y - sv_cent_1.y;
		sv_diff_1.at<double>(i, 2) = p3d_1_exp[i].z - sv_cent_1.z;
		numAdd += 3;
	}

	for (int i = 0; i < length; ++i) {
		sv_diff_2.at<double>(i, 0) = p3d_2_exp[i].x - sv_cent_2.x;
		sv_diff_2.at<double>(i, 1) = p3d_2_exp[i].y - sv_cent_2.y;
		sv_diff_2.at<double>(i, 2) = p3d_2_exp[i].z - sv_cent_2.z;
		numAdd += 3;
	}

	for (int i = 0; i < length; ++i) {
		sv_diff_3.at<double>(i, 0) = p3d_3_exp[i].x - sv_cent_3.x;
		sv_diff_3.at<double>(i, 1) = p3d_3_exp[i].y - sv_cent_3.y;
		sv_diff_3.at<double>(i, 2) = p3d_3_exp[i].z - sv_cent_3.z;
		numAdd += 3;
	}

	// Compute the covariance matrices
	Mat sv_corr_12 (3, 3, CV_64FC1);
	Mat sv_corr_23 (3, 3, CV_64FC1);
	Mat sv_corr_31 (3, 3, CV_64FC1);

	sv_corr_12 = sv_diff_1.t() * sv_diff_2;
	sv_corr_23 = sv_diff_2.t() * sv_diff_3;
	sv_corr_31 = sv_diff_3.t() * sv_diff_1;
	numMulAdd += (length-1) * 3 * 3;
	numMul += 3 * 3;

	// Compute the singular value decompositions
	Mat w;
	Mat svd_U_12, svd_V_12t;
	Mat svd_U_23, svd_V_23t;
	Mat svd_U_31, svd_V_31t;
	SVDecomp (sv_corr_12, w, svd_U_12, svd_V_12t);
	SVDecomp (sv_corr_23, w, svd_U_23, svd_V_23t);
	SVDecomp (sv_corr_31, w, svd_U_31, svd_V_31t);
	numSVD += 3;

	Mat svd_V_12 = svd_V_12t.t();
	Mat svd_U_12t = svd_U_12.t();
	Mat svd_V_23 = svd_V_23t.t();
	Mat svd_U_23t = svd_U_23.t();
	Mat svd_V_31 = svd_V_31t.t();
	Mat svd_U_31t = svd_U_31.t();

	svd_rotation (svd_V_12, svd_U_12t, sv_r_12);
	svd_rotation (svd_V_23, svd_U_23t, sv_r_23);
	svd_rotation (svd_V_31, svd_U_31t, sv_r_31);

	Mat sv_cent_1v (1, 3, CV_64FC1);
	Mat sv_cent_2v (1, 3, CV_64FC1);
	Mat sv_cent_3v (1, 3, CV_64FC1);

	sv_cent_1v.at<double>(0, 0) = sv_cent_1.x;
	sv_cent_1v.at<double>(0, 1) = sv_cent_1.y;
	sv_cent_1v.at<double>(0, 2) = sv_cent_1.z;

	sv_cent_2v.at<double>(0, 0) = sv_cent_2.x;
	sv_cent_2v.at<double>(0, 1) = sv_cent_2.y;
	sv_cent_2v.at<double>(0, 2) = sv_cent_2.z;

	sv_cent_3v.at<double>(0, 0) = sv_cent_3.x;
	sv_cent_3v.at<double>(0, 1) = sv_cent_3.y;
	sv_cent_3v.at<double>(0, 2) = sv_cent_3.z;

	sv_t_12 = sv_cent_2v - (sv_r_12 * sv_cent_1v.t()).t();
	sv_t_23 = sv_cent_3v - (sv_r_23 * sv_cent_2v.t()).t();
	sv_t_31 = sv_cent_1v - (sv_r_31 * sv_cent_3v.t()).t();
	numMulAdd += (2 * 3) * 3;
	numMul += 3 * 3;
	numAdd += 3 * 3;

/*
	cout << "estimation_rot_trans:" << endl;
	cout << " #Add: " << numAdd;
	cout << " #Mul: " << numMul;
	cout << " #MulAdd: " << numMulAdd;
	cout << " #SVDcomp: " << numSVD;
	cout << " #Div: " << numDiv;
	cout << endl;
*/
}

void intersection (const Mat &liste_p1, const Mat &liste_p2, const Mat &liste_p3,
				   const Mat &liste_azim1, const Mat &liste_azim2, const Mat &liste_azim3,
				   Mat &inter)
{

//	int numMul(0), numMulAdd(0), numAdd(0), numMatInv(0);

	Mat sum_v(3, 3, CV_64FC1, double(0));
	Mat sum_vp(3, 3, CV_64FC1, double(0));

	Mat v1(3, 3, CV_64FC1);
	Mat v2(3, 3, CV_64FC1);
	Mat v3(3, 3, CV_64FC1);
	Mat vp1(3, 1, CV_64FC1);
	Mat vp2(3, 1, CV_64FC1);
	Mat vp3(3, 1, CV_64FC1);

	v1 = Mat::eye(3, 3, CV_64FC1) - liste_azim1.t() * liste_azim1;
	numMul += 9;
	numAdd += 3 * 3;
	vp1 = v1 * liste_p1.t();
	numMul += 1;
	numMulAdd += 2;

	v2 = Mat::eye(3, 3, CV_64FC1) - liste_azim2.t() * liste_azim2;
	numMul += 9;
	numAdd += 3 * 3;
	vp2 = v2 * liste_p2.t();
	numMul += 1;
	numMulAdd += 2;

	v3 = Mat::eye(3, 3, CV_64FC1) - liste_azim3.t() * liste_azim3;
	numMul += 9;
	numAdd += 3 * 3;
	vp3 = v3 * liste_p3.t();
	numMul += 1;
	numMulAdd += 2;

	sum_v = v1 + v2 + v3;
	numAdd += 3 * 3 * 2;
	sum_vp = vp1 + vp2 + vp3;
	numAdd += 3 * 2;

	inter = (sum_v.inv() * sum_vp).t();
	numMatInv += 1;
	numMulAdd += 2 * 3;
	numMul += 1 * 3;

/*
	cout << "intersection:" << endl;
	cout << " #Mul: " << numMul << endl;
	cout << " #MulAdd: " << numMulAdd << endl;
	cout << " #Add: " << numAdd << endl;
	cout << " #MatInv: " << numMatInv << endl;
*/

}


void estimation_rayons (const vector<Point_t> &p3d_1, const vector<Point_t> &p3d_2, const vector<Point_t> &p3d_3,
						const Mat &sv_r_12, const Mat &sv_r_23, const Mat &sv_r_31,
						const Mat &sv_t_12, const Mat &sv_t_23, const Mat &sv_t_31,
						vector<double> &sv_u, vector<double> &sv_v, vector<double> &sv_w)
{
//	int numMul(0), numMulAdd(0), numDiv(0), numAdd(0), numSqrt(0);

	int longueur = p3d_1.size();

	Mat c1(1, 3, CV_64FC1, double(0));
	Mat c2(1, 3, CV_64FC1);
	Mat c3(1, 3, CV_64FC1);

	c2 = c1 + sv_t_12;
	numAdd += 3;
	c3 = c2 + (sv_r_12 * sv_t_23.t()).t();
	numMulAdd += 2 * 3;
	numMul += 3;
	numAdd += 3;

	for (int i=0; i<longueur; ++i) {
		sv_u[i]=0;
		sv_v[i]=0;
		sv_w[i]=0;
	}

	for (int i=0; i<longueur; ++i) {

		Mat azim1(1, 3, CV_64FC1);
		Mat azim2(1, 3, CV_64FC1);
		Mat azim3(1, 3, CV_64FC1);

		azim1.at<double>(0, 0) = p3d_1[i].x;
		azim1.at<double>(0, 1) = p3d_1[i].y;
		azim1.at<double>(0, 2) = p3d_1[i].z;

		azim2.at<double>(0, 0) = p3d_2[i].x;
		azim2.at<double>(0, 1) = p3d_2[i].y;
		azim2.at<double>(0, 2) = p3d_2[i].z;

		azim3.at<double>(0, 0) = p3d_3[i].x;
		azim3.at<double>(0, 1) = p3d_3[i].y;
		azim3.at<double>(0, 2) = p3d_3[i].z;

		azim2 = azim2 * sv_r_23;
		azim2 = azim2 * sv_r_31;
		azim3 = azim3 * sv_r_31;
		numMulAdd += 2 * 3 * 3;
		numMul += 3 * 3;

		Mat inter(1, 3, CV_64FC1);

		intersection (c1, c2, c3, azim1, azim2, azim3, inter);

		Mat inter1(1, 3, CV_64FC1);
		Mat inter2(1, 3, CV_64FC1);
		Mat inter3(1, 3, CV_64FC1);

		inter1 = c1 + (((inter - c1) * azim1.t()) / (azim1.dot(azim1))) * azim1;
		numMulAdd += (2 + 2);
		numAdd += (3 + 3);
		numDiv += 1;
		numMul += 3 + 1 + 1;
		inter2 = c2 + (((inter - c2) * azim2.t()) / (azim2.dot(azim2))) * azim2;
		numMulAdd += (2 + 2);
		numAdd += (3 + 3);
		numDiv += 1;
		numMul += 3 + 1 + 1;
		inter3 = c3 + (((inter - c3) * azim3.t()) / (azim3.dot(azim3))) * azim3;
		numMulAdd += (2 + 2);
		numAdd += (3 + 3);
		numDiv += 1;
		numMul += 3 + 1 + 1;


		sv_u[i] = norm(inter1 - c1);
		numAdd += 3;
		numMulAdd += 2;
		numMul += 1;
		numSqrt += 1;
		sv_v[i] = norm(inter2 - c2);
		numAdd += 3;
		numMulAdd += 2;
		numMul += 1;
		numSqrt += 1;
		sv_w[i] = norm(inter3 - c3);
		numAdd += 3;
		numMulAdd += 2;
		numMul += 1;
		numSqrt += 1;
	}

/*
	cout << "estimation_rayons:" << endl;
	cout << " #Mul: " << numMul << endl;
	cout << " #MulAdd: " << numMulAdd << endl;
	cout << " #Div: " << numDiv << endl;
	cout << " #Add: " << numAdd << endl;
	cout << " #Sqrt: " << numSqrt << endl;
*/

}

void pose_scene (const vector<Point_t> &p3d_1, const vector<Point_t> &p3d_2, const vector<Point_t> &p3d_3,
				 const Mat &sv_r_12, const Mat &sv_r_23, const Mat &sv_r_31,
				 const Mat &sv_t_12, const Mat &sv_t_23, const Mat &sv_t_31,
				 vector<Point_t> &sv_scene)
{
//	int numMul(0), numMulAdd(0), numAdd(0);

	int longueur = p3d_1.size();

	Mat c1(1, 3, CV_64FC1, double(0));
	Mat c2(1, 3, CV_64FC1);
	Mat c3(1, 3, CV_64FC1);

	c2 = c1 + sv_t_12;
	numAdd += 3;
	c3 = c2 + (sv_r_12 * sv_t_23.t()).t();
	numMulAdd += 2 * 3;
	numMul += 1 * 3;
	numAdd += 3;

	for (int i=0; i<longueur; ++i) {
		sv_scene[i].x=0;
		sv_scene[i].y=0;
		sv_scene[i].z=0;
	}

	for (int i=0; i<longueur; ++i) {

		Mat azim1(1, 3, CV_64FC1);
		Mat azim2(1, 3, CV_64FC1);
		Mat azim3(1, 3, CV_64FC1);

		azim1.at<double>(0, 0) = p3d_1[i].x;
		azim1.at<double>(0, 1) = p3d_1[i].y;
		azim1.at<double>(0, 2) = p3d_1[i].z;

		azim2.at<double>(0, 0) = p3d_2[i].x;
		azim2.at<double>(0, 1) = p3d_2[i].y;
		azim2.at<double>(0, 2) = p3d_2[i].z;

		azim3.at<double>(0, 0) = p3d_3[i].x;
		azim3.at<double>(0, 1) = p3d_3[i].y;
		azim3.at<double>(0, 2) = p3d_3[i].z;

		azim2 = azim2 * sv_r_23;
		numMulAdd += 2 * 3;
		numMul += 1 * 3;
		azim2 = azim2 * sv_r_31;
		numMulAdd += 2 * 3;
		numMul += 1 * 3;
		azim3 = azim3 * sv_r_31;
		numMulAdd += 2 * 3;
		numMul += 1 * 3;

		Mat inter(1, 3, CV_64FC1);

		intersection (c1, c2, c3, azim1, azim2, azim3, inter);

		sv_scene[i].x = inter.at<double>(0, 0);
		sv_scene[i].y = inter.at<double>(0, 1);
		sv_scene[i].z = inter.at<double>(0, 2);

	}
/*
	cout << "pose_scene:" << endl;
	cout << " #Mul:" << numMul << endl;
	cout << " #MulAdd: " << numMulAdd << endl;
	cout << " #Add: " << numAdd << endl;
*/

}

int main() {

	int iterations = 50;

	vector<Point_t> p3d_1, p3d_2, p3d_3;
	vector<Point_t> sv_scene;

	Mat sv_r_12(3, 3, CV_64FC1);
	Mat sv_r_23(3, 3, CV_64FC1);
	Mat sv_r_31(3, 3, CV_64FC1);
	Mat sv_t_12(1, 3, CV_64FC1);
	Mat sv_t_23(1, 3, CV_64FC1);
	Mat sv_t_31(1, 3, CV_64FC1);

	if (loadVectors (p3d_1, p3d_2, p3d_3)){
		cerr << "Error loading file" << endl;
		exit(1);
	}

	vector<double> sv_u, sv_v, sv_w;

	int length = p3d_1.size();

	for (int i=0; i<length; ++i) {
		sv_u.push_back(1);
		sv_v.push_back(1);
		sv_w.push_back(1);
	}

	for (int i=0; i<length; ++i) {
		Point_t p;
		p.x = 0;
		p.y = 0;
		p.z = 0;
		sv_scene.push_back(p);
	}

	for (int i=0; i<iterations; ++i) {
		estimation_rot_trans (p3d_1, p3d_2, p3d_3, sv_u, sv_v, sv_w, sv_r_12, sv_r_23, sv_r_31, sv_t_12, sv_t_23, sv_t_31);
		estimation_rayons (p3d_1, p3d_2, p3d_3, sv_r_12, sv_r_23, sv_r_31, sv_t_12, sv_t_23, sv_t_31, sv_u, sv_v, sv_w);
	}

	pose_scene (p3d_1, p3d_2, p3d_3, sv_r_12, sv_r_23, sv_r_31, sv_t_12, sv_t_23, sv_t_31, sv_scene);

	if (writeVector ("/home/scanvandev/ScanVan/PoseEstimation/C++/sv_scene.txt", sv_scene)) {
		cerr << "Error writing file" << endl;
		exit(1);
	}

	cout << "Finished computing." << endl;

	cout << " Length: " << length << endl;
	cout << " #Add: " << numAdd << endl;
	cout << " #Mul/MulAdd: " << numMul + numMulAdd << endl;
	cout << " #Div: " << numDiv << endl;
	cout << " #Sqrt: " << numSqrt << endl;
	cout << " #SVD: " << numSVD << endl;
	cout << " #Det: " << numDeterm << endl;
	cout << " #MatInv: " << numMatInv << endl;

	return 0;
}






