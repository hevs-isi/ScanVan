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



#endif /* SRC_ESTIMATION_HPP_ */
