#ifndef SRC_MAT33_HPP_
#define SRC_MAT33_HPP_

#include "VecPoints.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

template<typename T>
class Mat33 {
private:
	T mat[3][3] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
public:
	Mat33() {};
	Mat33(T a00, T a01, T a02, T a10, T a11, T a12, T a20, T a21, T a22):
		mat{{a00, a01, a02}, {a10, a11, a12}, {a20, a21, a22}} {};
	void mul_vecPt_vecP (VecPoints<T> &a, VecPoints<T> &b);
	void mul_P_2_P (point_t<T> &b, point_t<T> &c);
	void svd (Mat33<T> &ut, Mat33<T> &v);
	Mat33<T> & operator=(const Mat33<T> &a);
	void svd_rotation (Mat33<T> &v, Mat33<T> &u);
	virtual ~Mat33() {};
	friend std::ostream & operator <<(std::ostream & out, const Mat33<T> &a) {
		out << "[[" << a.mat[0][0] << " " << a.mat[0][1] << " " << a.mat[0][2] << "]" << std::endl;
		out << " [" << a.mat[1][0] << " " << a.mat[1][1] << " " << a.mat[1][2] << "]" << std::endl;
		out << " [" << a.mat[2][0] << " " << a.mat[2][1] << " " << a.mat[2][2] << "]]" << std::endl;
		return out;
	}
};


template <typename T>
inline void Mat33<T>::mul_vecPt_vecP(VecPoints<T> &a, VecPoints<T> &b){
	if (a.size() != b.size()) {
		throw std::runtime_error("The size of the vector of points transposed does not match to the size of the vector of points.");
	}
	T a00{0},a01{0},a02{0},a10{0},a11{0},a12{0},a20{0},a21{0},a22{0};
	for (size_t i{0}; i<a.size(); ++i){
		point_t<T> x{a[i]};
		point_t<T> y{b[i]};
		a00 += x[0]*y[0];
		a01 += x[0]*y[1];
		a02 += x[0]*y[2];
		a10 += x[1]*y[0];
		a11 += x[1]*y[1];
		a12 += x[1]*y[2];
		a20 += x[2]*y[0];
		a21 += x[2]*y[1];
		a22 += x[2]*y[2];
	}
	mat[0][0]=a00;
	mat[0][1]=a01;
	mat[0][2]=a02;
	mat[1][0]=a10;
	mat[1][1]=a11;
	mat[1][2]=a12;
	mat[2][0]=a20;
	mat[2][1]=a21;
	mat[2][2]=a22;
}

template <typename T>
Mat33<T> & Mat33<T>::operator=(const Mat33<T> &a) {
	if (this != &a){
		this->mat[0][0]=a.mat[0][0];
		this->mat[0][1]=a.mat[0][1];
		this->mat[0][1]=a.mat[0][1];
		this->mat[1][0]=a.mat[1][0];
		this->mat[1][1]=a.mat[1][1];
		this->mat[1][1]=a.mat[1][1];
		this->mat[2][0]=a.mat[2][0];
		this->mat[2][1]=a.mat[2][1];
		this->mat[2][1]=a.mat[2][1];
	}
	return *this;
}


template <typename T>
void Mat33<T>::svd (Mat33<T> &ut, Mat33<T> &v){
	using namespace cv;

	Mat m(3, 3, CV_64FC1);
	m.at<double>(0,0)=mat[0][0];
	m.at<double>(0,1)=mat[0][1];
	m.at<double>(0,2)=mat[0][2];
	m.at<double>(1,0)=mat[1][0];
	m.at<double>(1,1)=mat[1][1];
	m.at<double>(1,2)=mat[1][2];
	m.at<double>(2,0)=mat[2][0];
	m.at<double>(2,1)=mat[2][1];
	m.at<double>(2,2)=mat[2][2];

	Mat U, Vt, w;
	SVDecomp (m, w, U, Vt);

	ut.mat[0][0]=U.at<double>(0,0);
	ut.mat[0][1]=U.at<double>(1,0);
	ut.mat[0][2]=U.at<double>(2,0);
	ut.mat[1][0]=U.at<double>(0,1);
	ut.mat[1][1]=U.at<double>(1,1);
	ut.mat[1][2]=U.at<double>(2,1);
	ut.mat[2][0]=U.at<double>(0,2);
	ut.mat[2][1]=U.at<double>(1,2);
	ut.mat[2][2]=U.at<double>(2,2);

	v.mat[0][0]=Vt.at<double>(0,0);
	v.mat[0][1]=Vt.at<double>(1,0);
	v.mat[0][2]=Vt.at<double>(2,0);
	v.mat[1][0]=Vt.at<double>(0,1);
	v.mat[1][1]=Vt.at<double>(1,1);
	v.mat[1][2]=Vt.at<double>(2,1);
	v.mat[2][0]=Vt.at<double>(0,2);
	v.mat[2][1]=Vt.at<double>(1,2);
	v.mat[2][2]=Vt.at<double>(2,2);

}


template <typename T>
void Mat33<T>::svd_rotation (Mat33<T> &v, Mat33<T> &u){

	T a00{0},a01{0},a02{0},a10{0},a11{0},a12{0},a20{0},a21{0},a22{0};

	a00 = v.mat[0][0] * u.mat[0][0] + v.mat[0][1] * u.mat[1][0] + v.mat[0][2] * u.mat[2][0];
	a01 = v.mat[0][0] * u.mat[0][1] + v.mat[0][1] * u.mat[1][1] + v.mat[0][2] * u.mat[2][1];
	a02 = v.mat[0][0] * u.mat[0][2] + v.mat[0][1] * u.mat[1][2] + v.mat[0][2] * u.mat[2][2];

	a10 = v.mat[1][0] * u.mat[0][0] + v.mat[1][1] * u.mat[1][0] + v.mat[1][2] * u.mat[2][0];
	a11 = v.mat[1][0] * u.mat[0][1] + v.mat[1][1] * u.mat[1][1] + v.mat[1][2] * u.mat[2][1];
	a12 = v.mat[1][0] * u.mat[0][2] + v.mat[1][1] * u.mat[1][2] + v.mat[1][2] * u.mat[2][2];

	a20 = v.mat[2][0] * u.mat[0][0] + v.mat[2][1] * u.mat[1][0] + v.mat[2][2] * u.mat[2][0];
	a21 = v.mat[2][0] * u.mat[0][1] + v.mat[2][1] * u.mat[1][1] + v.mat[2][2] * u.mat[2][1];
	a22 = v.mat[2][0] * u.mat[0][2] + v.mat[2][1] * u.mat[1][2] + v.mat[2][2] * u.mat[2][2];

	T det = a00 * a11 * a22 + a01 * a12 * a20 + a02 * a10 * a21 - a02 * a11 * a20 - a01 * a10 * a22 - a00 * a12 * a21;

	T vv02{v.mat[0][2] * det};
	T vv12{v.mat[1][2] * det};
	T vv22{v.mat[2][2] * det};

	mat[0][0] = v.mat[0][0] * u.mat[0][0] + v.mat[0][1] * u.mat[1][0] + vv02 * u.mat[2][0];
	mat[0][1] = v.mat[0][0] * u.mat[0][1] + v.mat[0][1] * u.mat[1][1] + vv02 * u.mat[2][1];
	mat[0][2] = v.mat[0][0] * u.mat[0][2] + v.mat[0][1] * u.mat[1][2] + vv02 * u.mat[2][2];

	mat[1][0] = v.mat[1][0] * u.mat[0][0] + v.mat[1][1] * u.mat[1][0] + vv12 * u.mat[2][0];
	mat[1][1] = v.mat[1][0] * u.mat[0][1] + v.mat[1][1] * u.mat[1][1] + vv12 * u.mat[2][1];
	mat[1][2] = v.mat[1][0] * u.mat[0][2] + v.mat[1][1] * u.mat[1][2] + vv12 * u.mat[2][2];

	mat[2][0] = v.mat[2][0] * u.mat[0][0] + v.mat[2][1] * u.mat[1][0] + vv22 * u.mat[2][0];
	mat[2][1] = v.mat[2][0] * u.mat[0][1] + v.mat[2][1] * u.mat[1][1] + vv22 * u.mat[2][1];
	mat[2][2] = v.mat[2][0] * u.mat[0][2] + v.mat[2][1] * u.mat[1][2] + vv22 * u.mat[2][2];

}

template <typename T>
void Mat33<T>::mul_P_2_P (point_t<T> &b, point_t<T> &to_c){
	to_c[0] = mat[0][0] * b[0] + mat[0][1] * b[1] + mat[0][2] * b[2];
	to_c[1] = mat[1][0] * b[0] + mat[1][1] * b[1] + mat[1][2] * b[2];
	to_c[2] = mat[2][0] * b[0] + mat[2][1] * b[1] + mat[2][2] * b[2];
}


#endif /* SRC_MAT33_HPP_ */
