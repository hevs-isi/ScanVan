#ifndef SRC_MAT_33_HPP_
#define SRC_MAT_33_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "Points.hpp"

template <typename T>
class Mat_33 {
private:
	T **mat;
public:
	Mat_33();
	Mat_33(T a00, T a01, T a02, T a10, T a11, T a12, T a20, T a21, T a22);
	Mat_33(std::initializer_list<T> a0, std::initializer_list<T> a1, std::initializer_list<T> a2);
	Mat_33(const Points<T> &c1, const Points<T> &c2, const Points<T> &c3);
	Mat_33(const Mat_33<T> &obj);
	Mat_33(Mat_33<T> &&obj);
	void svd (Mat_33<T> &ut, Mat_33<T> &v) const;
	void svd_rotation (Mat_33<T> &v, Mat_33<T> &u);
	Mat_33<T> inv () const;
	Mat_33<T> & operator=(const Mat_33<T> &a);
	Mat_33<T> & operator=(Mat_33<T> &&a);
	Points<T> operator*(const Points<T> &b) const;
	Mat_33<T> operator+(const Mat_33<T> &a) const;
	const T * operator[](const size_t i) const;
	virtual ~Mat_33();
	friend std::ostream & operator <<(std::ostream & out, const Mat_33<T> &a) {
		out << "[[" << a.mat[0][0] << " " << a.mat[0][1] << " " << a.mat[0][2] << "]" << std::endl;
		out << " [" << a.mat[1][0] << " " << a.mat[1][1] << " " << a.mat[1][2] << "]" << std::endl;
		out << " [" << a.mat[2][0] << " " << a.mat[2][1] << " " << a.mat[2][2] << "]]" << std::endl;
		return out;
	}
};

template <typename T>
inline Mat_33<T>::Mat_33(){
	T *p1 = new T[3]{0, 0, 0};
	T *p2 = new T[3]{0, 0, 0};
	T *p3 = new T[3]{0, 0, 0};
	mat = new T *[3]{p1, p2, p3};
}

template <typename T>
inline Mat_33<T>::Mat_33(T a00, T a01, T a02, T a10, T a11, T a12, T a20, T a21, T a22){
	T *p1 = new T[3]{a00, a01, a02};
	T *p2 = new T[3]{a10, a11, a12};
	T *p3 = new T[3]{a20, a21, a22};
	mat = new T *[3]{p1, p2, p3};
}

template <typename T>
inline Mat_33<T>::Mat_33(std::initializer_list<T> a0, std::initializer_list<T> a1, std::initializer_list<T> a2){
	T *p1 = new T[3]{0, 0, 0};
	T *p2 = new T[3]{0, 0, 0};
	T *p3 = new T[3]{0, 0, 0};
	mat = new T *[3]{p1, p2, p3};
	int index{};
	for (const T &x : a0) {
		mat[0][index++] = x;
	}
	index = 0;
	for (const T &x : a1) {
		mat[1][index++] = x;
	}
	index = 0;
	for (const T &x : a2) {
		mat[2][index++] = x;
	}
}

template <typename T>
inline Mat_33<T>::Mat_33(const Points<T> &c1, const Points<T> &c2, const Points<T> &c3) {
	T *p1 = new T[3]{c1[0], c1[1], c1[2]};
	T *p2 = new T[3]{c2[0], c2[1], c2[2]};
	T *p3 = new T[3]{c3[0], c3[1], c3[2]};
	mat = new T *[3]{p1, p2, p3};
}

template <typename T>
inline Mat_33<T>::Mat_33(const Mat_33<T> &obj){
	T *p1 = new T[3]{obj.mat[0][0], obj.mat[0][1], obj.mat[0][2]};
	T *p2 = new T[3]{obj.mat[1][0], obj.mat[1][1], obj.mat[1][2]};
	T *p3 = new T[3]{obj.mat[2][0], obj.mat[2][1], obj.mat[2][2]};
	mat = new T *[3]{p1, p2, p3};
}

template <typename T>
inline Mat_33<T>::Mat_33(Mat_33<T> &&obj){
	mat = obj.mat;
	obj.mat = nullptr;
}

template <typename T>
inline void Mat_33<T>::svd (Mat_33<T> &ut, Mat_33<T> &v) const{
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
inline void Mat_33<T>::svd_rotation (Mat_33<T> &v, Mat_33<T> &u){

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
inline Mat_33<T> Mat_33<T>::inv() const {

	using namespace cv;

	Mat m(3, 3, CV_64FC1);
	m.at<double>(0, 0) = mat[0][0];
	m.at<double>(0, 1) = mat[0][1];
	m.at<double>(0, 2) = mat[0][2];
	m.at<double>(1, 0) = mat[1][0];
	m.at<double>(1, 1) = mat[1][1];
	m.at<double>(1, 2) = mat[1][2];
	m.at<double>(2, 0) = mat[2][0];
	m.at<double>(2, 1) = mat[2][1];
	m.at<double>(2, 2) = mat[2][2];

	Mat res(3, 3, CV_64FC1);
	res = m.inv();

	Mat_33<T> temp {res.at<double>(0, 0), res.at<double>(0, 1), res.at<double>(0, 2),
					res.at<double>(1, 0), res.at<double>(1, 1), res.at<double>(1, 2),
					res.at<double>(2, 0), res.at<double>(2, 1), res.at<double>(2, 2)};

	return temp;
}

template <typename T>
inline Mat_33<T> & Mat_33<T>::operator=(const Mat_33<T> &a) {
	if (this != &a){
		this->mat[0][0]=a.mat[0][0];
		this->mat[0][1]=a.mat[0][1];
		this->mat[0][2]=a.mat[0][2];
		this->mat[1][0]=a.mat[1][0];
		this->mat[1][1]=a.mat[1][1];
		this->mat[1][2]=a.mat[1][2];
		this->mat[2][0]=a.mat[2][0];
		this->mat[2][1]=a.mat[2][1];
		this->mat[2][2]=a.mat[2][2];
	}
	return *this;
}

template <typename T>
inline Mat_33<T> & Mat_33<T>::operator=(Mat_33<T> &&a) {
	if (this != &a){
		delete[] mat[0];
		delete[] mat[1];
		delete[] mat[2];
		delete[] mat;
		mat = a.mat;
		a.mat = nullptr;
	}
	return *this;
}

template <typename T>
inline Points<T> Mat_33<T>::operator*(const Points<T> &b) const{
	T to_c0{mat[0][0] * b[0] + mat[0][1] * b[1] + mat[0][2] * b[2]};
	T to_c1{mat[1][0] * b[0] + mat[1][1] * b[1] + mat[1][2] * b[2]};
	T to_c2{mat[2][0] * b[0] + mat[2][1] * b[1] + mat[2][2] * b[2]};
	Points<T> temp{to_c0, to_c1, to_c2};
	return temp;
}

template <typename T>
inline Mat_33<T> Mat_33<T>::operator+(const Mat_33<T> &a) const{
	Mat_33<T> temp {mat[0][0] + a.mat[0][0], mat[0][1] + a.mat[0][1], mat[0][2] + a.mat[0][2],
					mat[1][0] + a.mat[1][0], mat[1][1] + a.mat[1][1], mat[1][2] + a.mat[1][2],
					mat[2][0] + a.mat[2][0], mat[2][1] + a.mat[2][1], mat[2][2] + a.mat[2][2]};
	return temp;
}

template <typename T>
inline const T * Mat_33<T>::operator[](const size_t i) const{
	if (i < 3) {
		return mat[i];
	} else {
		throw std::out_of_range("Invalid access to Mat_33 elements.");
	}
}

template <typename T>
inline Mat_33<T>::~Mat_33(){
	if (mat!=nullptr) {
		delete[] mat[0];
		delete[] mat[1];
		delete[] mat[2];
	}
	delete[] mat;
}

#endif /* SRC_MAT_33_HPP_ */
