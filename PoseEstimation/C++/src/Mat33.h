#ifndef SRC_MAT33_H_
#define SRC_MAT33_H_

#include "VecPoints.h"

template<typename T>
class Mat33 {
private:
	T mat[3][3] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
public:
	Mat33() {};
	Mat33(T a00, T a01, T a02, T a10, T a11, T a12, T a20, T a21, T a22):
		mat{{a00, a01, a02}, {a10, a11, a12}, {a20, a21, a22}} {};
	void mul_vecPt_vecP (VecPoints<T> &a, VecPoints<T> &b);
	~Mat33() {};
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
		a00 += x[2]*y[0];
		a01 += x[2]*y[1];
		a02 += x[2]*y[2];
		a10 += x[1]*y[0];
		a11 += x[1]*y[1];
		a12 += x[1]*y[2];
		a20 += x[0]*y[0];
		a21 += x[0]*y[1];
		a22 += x[0]*y[2];
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


#endif /* SRC_MAT33_H_ */
