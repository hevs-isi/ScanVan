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
// Takes as inputs p3d_1, p3d_2, p3d_3, sv_u, sv_v, sv_w
// and generates outputs sv_r_12, sv_r_23, sv_r_31 and sv_t_12,sv_t_23 and sv_t_31
{
	// Check that sizes are all the same
	if (!((p3d_1.size() == p3d_2.size()) &&
	    (p3d_2.size() == p3d_3.size()) &&
	    (p3d_3.size() == sv_u.size()) &&
	    (sv_u.size() == sv_v.size()) &&
	    (sv_v.size() == sv_w.size()))) {

		throw std::runtime_error ("Sizes of the vector of points in estimation_rot_trans do not match");
	}

	// Multiply each point of the vector by the corresponding scalar
	Vec_Points<T> p3d_1_exp {p3d_1 * sv_u};
	Vec_Points<T> p3d_2_exp {p3d_2 * sv_v};
	Vec_Points<T> p3d_3_exp {p3d_3 * sv_w};

	// Calculates the centers of the vector of points
	Points<T> sv_cent_1 {p3d_1_exp.mean()};
	Points<T> sv_cent_2 {p3d_2_exp.mean()};
	Points<T> sv_cent_3 {p3d_3_exp.mean()};

	// Calculates the distances to the centers of the vector of points
	Vec_Points<T> sv_diff_1 {p3d_1_exp - sv_cent_1};
	Vec_Points<T> sv_diff_2 {p3d_2_exp - sv_cent_2};
	Vec_Points<T> sv_diff_3 {p3d_3_exp - sv_cent_3};

	// Multiply vector of points transposed by vector of points
	Mat_33<T> sv_corr_12 {sv_diff_1 * sv_diff_2};
	Mat_33<T> sv_corr_23 {sv_diff_2 * sv_diff_3};
	Mat_33<T> sv_corr_31 {sv_diff_3 * sv_diff_1};

	// Matrices for SVD computation
	Mat_33<T> svd_U_12t{}, svd_U_23t{}, svd_U_31t{};
	Mat_33<T> svd_V_12{}, svd_V_23{}, svd_V_31{};

	// Computation of SVD
	sv_corr_12.svd(svd_U_12t, svd_V_12);
	sv_corr_23.svd(svd_U_23t, svd_V_23);
	sv_corr_31.svd(svd_U_31t, svd_V_31);

	// Call the svd_rotation function
	sv_r_12.svd_rotation(svd_V_12, svd_U_12t);
	sv_r_23.svd_rotation(svd_V_23, svd_U_23t);
	sv_r_31.svd_rotation(svd_V_31, svd_U_31t);

	// Computation of translation vectors
	sv_t_12 = sv_cent_2 - (sv_r_12 * sv_cent_1);
	sv_t_23 = sv_cent_3 - (sv_r_23 * sv_cent_2);
	sv_t_31 = sv_cent_1 - (sv_r_31 * sv_cent_3);

}

template <typename T>
inline Points<T> intersection (const Mat_33<T> &c, const Mat_33<T> &azim){
// Takes as input matrices c and azim and returns a point

	// For accumulating the results
	std::vector<Mat_33<T>> v{};
	std::vector<Points<T>> vp{};

	for (int i{0}; i<3; ++i) {
		// takes row i from azim
		T a0 = azim[i][0];
		T a1 = azim[i][1];
		T a2 = azim[i][2];

		// computes identity(3) - azim[i]' * azim[i]
		Mat_33<T> v1{1 - a0*a0 ,    -a0*a1 ,    -a0*a2,
						-a1*a0 , 1 - a1*a1 ,    -a1*a2,
						-a2*a0 ,    -a2*a1 , 1 - a2*a2};

		v.push_back(v1);

		// takes the row of c
		Points<T> row {c[i][0], c[i][1], c[i][2]};

		Points<T> vp1 {v1 * row};

		vp.push_back(vp1);
	}

	Mat_33<T> sum_v = v[0] + v[1] + v[2];
	Points<T> sum_vp = vp[0] + vp[1] + vp[2];

	// Computes the inverse of the matrix
	Mat_33<T> sum_v_inv {sum_v.inv()};

	Points<T> inter {sum_v_inv * sum_vp};

	return inter;
}

template <typename T>
void estimation_rayons (const Vec_Points<T> &p3d_1, const Vec_Points<T> &p3d_2, const Vec_Points<T> &p3d_3,
						const Mat_33<T> &sv_r_12, const Mat_33<T> &sv_r_23, const Mat_33<T> &sv_r_31,
						const Points<T> &sv_t_12, const Points<T> &sv_t_23, const Points<T> &sv_t_31,
						std::vector<T> &sv_u, std::vector<T> &sv_v, std::vector<T> &sv_w) {
// Takes as input p3d_1, p3d_2, p3d_3, sv_r_12, sv_r_23, sv_r_31, sv_t_12, sv_t_23, sv_t_31
// and generates as output sv_u, sv_v and sv_w

	if (!((p3d_1.size() == p3d_2.size()) &&
	      (p3d_2.size() == p3d_3.size()))) {
		throw std::runtime_error ("Size of the vectors in estimation_rayons do not match.");
	}

	size_t longueur = p3d_1.size();

	Points<T> c1 {0, 0, 0};
	Points<T> c2 {sv_t_12}; // c2 = c1 + sv_t_12
	Points<T> c3 {sv_t_12 + sv_r_12 * sv_t_23}; // c3 = c2 + sv_r_12 * sv_t_23

	sv_u.assign(longueur, 0);
	sv_v.assign(longueur, 0);
	sv_w.assign(longueur, 0);

	Vec_Points<T> azim1m {p3d_1};
	Vec_Points<T> azim2m {(p3d_2 * sv_r_23) * sv_r_31};
	Vec_Points<T> azim3m {p3d_3 * sv_r_31};

	for (size_t i{0}; i<longueur; ++i)
	{

		Points<T> azim1 {azim1m[i]};
		Points<T> azim2 {azim2m[i]};
		Points<T> azim3 {azim3m[i]};

		Mat_33<T> c {c1, c2, c3};
		Mat_33<T> azim { azim1, azim2, azim3 };

		Points<T> inter{};
		inter = intersection (c, azim);

		// here I use scalar product of points
		T factor1 { ((inter - c1) * azim1) / (azim1 * azim1) };
		T factor2 { ((inter - c2) * azim2) / (azim2 * azim2) };
		T factor3 { ((inter - c3) * azim3) / (azim3 * azim3) };

		// here I use multiplication of a point by a scalar
		Points<T> inter1 { c1 + azim1 * factor1 };
		Points<T> inter2 { c2 + azim2 * factor2 };
		Points<T> inter3 { c3 + azim3 * factor3 };

		sv_u[i] = (inter1 - c1).norm();
		sv_v[i] = (inter2 - c2).norm();
		sv_w[i] = (inter3 - c3).norm();

	}
}

template <typename T>
void pose_scene (const Vec_Points<T> &p3d_1, const Vec_Points<T> &p3d_2, const Vec_Points<T> &p3d_3,
				 const Mat_33<T> &sv_r_12, const Mat_33<T> &sv_r_23, const Mat_33<T> &sv_r_31,
				 const Points<T> &sv_t_12, const Points<T> &sv_t_23, const Points<T> &sv_t_31,
				 Vec_Points<T> &sv_scene) {

	size_t longueur {p3d_1.size()};

	Points<T> c1 {0, 0, 0};
	Points<T> c2 {sv_t_12};	// c2 = c1 + sv_t_12
	Points<T> c3 {sv_t_12 + sv_r_12 * sv_t_23};	// c3 = c2 + sv_r_12 * sv_t_23

	Vec_Points<T> azim1m {p3d_1};
	Vec_Points<T> azim2m {(p3d_2 * sv_r_23) * sv_r_31};
	Vec_Points<T> azim3m {p3d_3 * sv_r_31};

	for (size_t i{0}; i < longueur; ++i) {

		Points<T> azim1 {azim1m[i]};
		Points<T> azim2 {azim2m[i]};
		Points<T> azim3 {azim3m[i]};

		Mat_33<T> c {c1, c2, c3};
		Mat_33<T> azim { azim1, azim2, azim3 };

		Points<T> inter{};
		inter = intersection (c, azim);

		sv_scene[i] = inter;
	}

}

template <typename T>
void pose_estimation (const Vec_Points<T> &p3d_1, const Vec_Points<T> &p3d_2, const Vec_Points<T> &p3d_3,
				 	  const size_t iterations,
					  Vec_Points<T> &sv_scene,
					  Mat_33<T> &sv_r_12, Mat_33<T> &sv_r_23, Mat_33<T> &sv_r_31,
					  Points<T> &sv_t_12, Points<T> &sv_t_23, Points<T> &sv_t_31) {

	std::vector<T> sv_u(p3d_1.size(),1);
	std::vector<T> sv_v(p3d_2.size(),1);
	std::vector<T> sv_w(p3d_3.size(),1);

	Points<T> zero {0, 0, 0};
	sv_scene.assign(p3d_1.size(), zero);

	for (size_t i{0}; i < iterations; ++i) {
		estimation_rot_trans (p3d_1, p3d_2, p3d_3,
							  sv_u, sv_v, sv_w,
							  sv_r_12, sv_r_23, sv_r_31,
							  sv_t_12, sv_t_23, sv_t_31);

		estimation_rayons (p3d_1, p3d_2, p3d_3,
						   sv_r_12, sv_r_23, sv_r_31,
						   sv_t_12, sv_t_23, sv_t_31,
						   sv_u, sv_v, sv_w);
	}

	pose_scene (p3d_1, p3d_2, p3d_3,
				sv_r_12, sv_r_23, sv_r_31,
				sv_t_12, sv_t_23, sv_t_31,
				sv_scene);

}

#endif /* SRC_ESTIMATION_HPP_ */
