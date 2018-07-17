#ifndef SRC_VECPOINTS_HPP_
#define SRC_VECPOINTS_HPP_

#include <array>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <limits>
#include "Mat_33.hpp"

template <typename T>
using point_t = std::array<T, 3>;

#ifndef TO_STRING_WITH_PRECISION
#define TO_STRING_WITH_PRECISION
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out{};
    out << std::setprecision(n) << a_value;
    return out.str();
}
#endif /* TO_STRING_WITH_PRECISION */

template <typename T>
class VecPoints {
private:
	std::vector<point_t<T>> points;
public:
	VecPoints(){};
	void push_back (const point_t<T> &p);
	void push_back (const double x, const double y, const double z);
	void pop_back();
	bool load_vecpoints (std::string path);
	bool save_vecpoints (std::string path) const;
	std::vector<point_t<T>> get_value() const;
	size_t size() const { return points.size();	}
	point_t<T> mean() const;
	point_t<T> operator[](const int i);
	VecPoints<T> & operator=(const VecPoints<T> &a);
	Mat_33<T> operator*(const VecPoints<T> &b) const;
	VecPoints<T> & operator*(const std::vector<T> &p);
	VecPoints<T> & operator-(const point_t<T> &p);
	friend std::ostream & operator <<(std::ostream & out, const VecPoints<T> &a) {
		for (const auto &x:a.points){
			out << "(" << x[0] << ", " << x[1] << ", " << x[2] << ")" << std::endl;
		}
		return out;
	}
	virtual ~VecPoints(){};
};


template <typename T>
void VecPoints<T>::push_back (const point_t<T> &p){
	points.push_back(p);
}

template <typename T>
void VecPoints<T>::push_back(double x, double y, double z){
	point_t<T> p1{x,y,z};
	push_back(p1);
}

template <typename T>
void VecPoints<T>::pop_back(){
	points.pop_back();
}

template <typename T>
bool VecPoints<T>::load_vecpoints(std::string path){
	std::ifstream inputFile{};
	std::string str(""), str1(""), str2(""), str3(""), str4("");
	std::string token;
	point_t<T> point{};
	int linenum{0};

	inputFile.open(path);
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
				token = str2.substr(0, str2.find(","));
				point[0] = stod(token, nullptr);
				str3 = str2.substr(str2.find(",") + 1, str2.length());
				token = str3.substr(0, str3.find(","));
				point[1] = stod(token, nullptr);
				str4 = str3.substr(str3.find(",") + 1, str3.length());
				token = str4.substr(0, str4.find("]"));
				point[2] = stod(token, nullptr);

				push_back(point);
				linenum++;
			}
		}
		inputFile.close();
	} else {
		std::cerr << "Error: Unable to open the file \"" << path << "\"";
		return true;
	}

	return false;
}

template <typename T>
bool VecPoints<T>::save_vecpoints(std::string path) const {


	std::ofstream outputFile{};

	outputFile.open (path);
	if (outputFile.is_open()) {
		outputFile << "[";
		for (size_t i{0}; i<points.size()-1; i++) {
			outputFile << "[" << to_string_with_precision(points[i][0], 10) << ", " <<
					to_string_with_precision(points[i][1], 10) << ", " <<
					to_string_with_precision(points[i][2], 10) << "];\n";
		}
		outputFile << "[" << to_string_with_precision(points[points.size()-1][0], 10) << ", " <<
							to_string_with_precision(points[points.size()-1][1], 10) << ", " <<
							to_string_with_precision(points[points.size()-1][2], 10) << "]];\n";
		outputFile.close();
	} else {
		std::cerr << "Error: Unable to open the file \"" << path << "\"";
		return true;
	}

	return false;
}

template <typename T>
std::vector<point_t<T>> VecPoints<T>::get_value() const{
	return points;
}

template <typename T>
inline point_t<T> VecPoints<T>::mean() const{
	T x{0}, y{0}, z{0};
	for (auto p:points) {
		x+=p[0];
		y+=p[1];
		z+=p[2];
	}
	size_t length{points.size()};
	point_t<T> p{x/length, y/length, z/length};
	return p;
}


template <typename T>
inline point_t<T> VecPoints<T>::operator [](const int i){
	point_t<T> p{points[i]};
	return p;
}

template <typename T>
VecPoints<T> & VecPoints<T>::operator=(const VecPoints<T> &a) {
	if (this != &a){
		this->points=a.points;
	}
	return *this;
}

template <typename T>
inline VecPoints<T> & VecPoints<T>::operator*(const std::vector<T> &p) {
	if (p.size() != points.size()) {
		throw std::runtime_error("The size of the vector of points does not match to the size of the vector.");
	}
	auto itr = p.begin();
	for (auto &x:points){
		x[0] *= *itr;
		x[1] *= *itr;
		x[2] *= *itr++;
	}
	return *this;
}

template<typename T>
inline Mat_33<T> VecPoints<T>::operator*(const VecPoints<T> &b) const {
	if (this->size() != b.size()) {
		throw std::runtime_error("The size of the vector of points transposed does not match to the size of the vector of points.");
	}
	T a00 { 0 }, a01 { 0 }, a02 { 0 }, a10 { 0 }, a11 { 0 }, a12 { 0 }, a20 { 0 }, a21 { 0 }, a22 { 0 };
	for (size_t i { 0 }; i < this->size(); ++i) {
		point_t<T> x { points[i] };
		point_t<T> y { b.points[i] };
		a00 += x[0] * y[0];
		a01 += x[0] * y[1];
		a02 += x[0] * y[2];
		a10 += x[1] * y[0];
		a11 += x[1] * y[1];
		a12 += x[1] * y[2];
		a20 += x[2] * y[0];
		a21 += x[2] * y[1];
		a22 += x[2] * y[2];
	}
	Mat_33<T> temp {a00, a01, a02, a10, a11, a12, a20, a21, a22};
	return temp;
}

template <typename T>
inline VecPoints<T> & VecPoints<T>::operator-(const point_t<T> &p) {
	for (auto &x:points) {
		x[0] -= p[0];
		x[1] -= p[1];
		x[2] -= p[2];
	}
	return *this;
}

#endif /* SRC_VECPOINTS_HPP_ */
