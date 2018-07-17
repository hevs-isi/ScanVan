#ifndef SRC_ESTIMATION_HPP_
#define SRC_ESTIMATION_HPP_

#include <stdio.h>
#include <iostream>
#include <vector>

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <experimental/filesystem>
#include <limits>
#include <unistd.h>
#include "Points.hpp"
#include "Mat_33.hpp"
#include "Vec_Points.hpp"

template <typename T>
void estimation_rot_trans (const Vec_Points<T> &p3d_1, const Vec_Points<T> &p3d_2, const Vec_Points<T> &p3d_3,
						   const std::vector<T> &sv_u, const std::vector<T> &sv_v, const std::vector<T> &sv_w,
						   Mat_33<T> &sv_r_12, Mat_33<T> &sv_r_23, Mat_33<T> &sv_r_31,
						   Points<T> &sv_t_12, Points<T> &sv_t_23, Points<T> &sv_t_31)
{

	Vec_Points<T> p3d_1_exp {p3d_1 * sv_u};
	Vec_Points<T> p3d_2_exp {p3d_2 * sv_v};
	Vec_Points<T> p3d_3_exp {p3d_3 * sv_w};

	Points<T> sv_cent_1 {p3d_1_exp.mean()};
	Points<T> sv_cent_2 {p3d_2_exp.mean()};
	Points<T> sv_cent_3 {p3d_3_exp.mean()};

	Vec_Points<T> sv_diff_1 {p3d_1_exp - sv_cent_1};
	Vec_Points<T> sv_diff_2 {p3d_2_exp - sv_cent_2};
	Vec_Points<T> sv_diff_3 {p3d_3_exp - sv_cent_3};

	Mat_33<T> sv_corr_12 {sv_diff_1 * sv_diff_2};
	Mat_33<T> sv_corr_23 {sv_diff_2 * sv_diff_3};
	Mat_33<T> sv_corr_31 {sv_diff_3 * sv_diff_1};

	Mat_33<T> svd_U_12t{}, svd_U_23t{}, svd_U_31t{};
	Mat_33<T> svd_V_12{}, svd_V_23{}, svd_V_31{};

	sv_corr_12.svd(svd_U_12t, svd_V_12);
	sv_corr_23.svd(svd_U_23t, svd_V_23);
	sv_corr_31.svd(svd_U_31t, svd_V_31);

	sv_r_12.svd_rotation(svd_V_12, svd_U_12t);
	sv_r_23.svd_rotation(svd_V_23, svd_U_23t);
	sv_r_31.svd_rotation(svd_V_31, svd_U_31t);

	sv_t_12 = sv_cent_2 - (sv_r_12 * sv_cent_1);
	sv_t_23 = sv_cent_3 - (sv_r_23 * sv_cent_2);
	sv_t_31 = sv_cent_1 - (sv_r_31 * sv_cent_3);

}

template <typename T>
void estimation_rayons (const Vec_Points<T> &p3d_1, const Vec_Points<T> &p3d_2, const Vec_Points<T> &p3d_3,
						const Mat_33<T> &sv_r_12, const Mat_33<T> &sv_r_23, const Mat_33<T> &sv_r_31,
						const Points<T> &sv_t_12, const Points<T> &sv_t_23, const Points<T> &sv_t_31,
						std::vector<T> &sv_u, std::vector<T> &sv_v, std::vector<T> &sv_w)
{

	size_t longueur = p3d_1.size();

	Points<T> c1 {0, 0, 0};
	Points<T> c2 {sv_t_12};
	Points<T> c3 {sv_t_12 + sv_r_12 * sv_t_23};

	sv_u.assign(longueur, 0);
	sv_v.assign(longueur, 0);
	sv_w.assign(longueur, 0);

/*
	std::cout << sv_u[0] << std::endl;
	std::cout << sv_u[longueur-1] << std::endl;
*/

/*
	Mat_33<T> a{0,1,2,3,4,5,6,7,8};
	std::cout << a[2][2] << std::endl;
*/


/*
	std::cout << c1 << std::endl;
	std::cout << c2 << std::endl;
	std::cout << c3 << std::endl;
*/

/*
	Mat c1(1, 3, CV_64FC1, double(0));
	Mat c2(1, 3, CV_64FC1);
	Mat c3(1, 3, CV_64FC1);

	c2 = c1 + sv_t_12;
	c3 = c2 + (sv_r_12 * sv_t_23.t()).t();

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

		Mat inter(1, 3, CV_64FC1);

		intersection (c1, c2, c3, azim1, azim2, azim3, inter);

		Mat inter1(1, 3, CV_64FC1);
		Mat inter2(1, 3, CV_64FC1);
		Mat inter3(1, 3, CV_64FC1);

		inter1 = c1 + (((inter - c1) * azim1.t()) / (azim1.dot(azim1))) * azim1;
		inter2 = c2 + (((inter - c2) * azim2.t()) / (azim2.dot(azim2))) * azim2;
		inter3 = c3 + (((inter - c3) * azim3.t()) / (azim3.dot(azim3))) * azim3;

		sv_u[i] = norm(inter1 - c1);
		sv_v[i] = norm(inter2 - c2);
		sv_w[i] = norm(inter3 - c3);
	}
*/

}





#endif /* SRC_ESTIMATION_HPP_ */
