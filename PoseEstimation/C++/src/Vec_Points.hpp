#ifndef SRC_VEC_POINTS_HPP_
#define SRC_VEC_POINTS_HPP_

#include <array>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <limits>
#include "Mat_33.hpp"
#include "Points.hpp"

template <typename T>
using vecpoint_t = std::vector<Points<T>>;

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
class Vec_Points {
private:
	vecpoint_t<T> *m_pV;
public:
	Vec_Points();
	Vec_Points(const Vec_Points<T> &obj);
	Vec_Points(Vec_Points<T> &&obj);
	Vec_Points(size_t longueur, Points<T> &p);
	Vec_Points(size_t longueur);
	void push_back (const Points<T> &p);
	void push_back (const T x, const T y, const T z);
	void pop_back();
	bool load_vecpoints (std::string &path);
	bool save_vecpoints (std::string path) const;
	void assign (size_t longueur, Points<T> &p);
	size_t size() const { return m_pV->size(); }
	Points<T> mean() const;
	const Points<T> & operator[](const size_t i) const;
	Points<T> & operator[](const size_t i);
	Vec_Points<T> & operator=(const Vec_Points<T> &a);
	Mat_33<T> operator*(const Vec_Points<T> &b) const;
	Vec_Points<T> operator*(const std::vector<T> &p) const;
	Vec_Points<T> operator*(const Mat_33<T> &a) const;
	Vec_Points<T> operator-(const Points<T> &p);

	friend std::ostream & operator <<(std::ostream & out, const Vec_Points<T> &a) {
		for (const auto &x:*(a.m_pV)){
			out << "(" << x[0] << ", " << x[1] << ", " << x[2] << ")" << std::endl;
		}
		return out;
	}
	virtual ~Vec_Points();
};

template <typename T>
inline Vec_Points<T>::Vec_Points() {
	m_pV = new vecpoint_t<T>{};
}

template <typename T>
inline Vec_Points<T>::Vec_Points(const Vec_Points<T> &obj){
	m_pV = new vecpoint_t<T>{};
	(*m_pV) = *(obj.m_pV);
}

template <typename T>
inline Vec_Points<T>::Vec_Points(Vec_Points<T> &&obj){
	m_pV = obj.m_pV;
	obj.m_pV = nullptr;
}

template <typename T>
inline Vec_Points<T>::Vec_Points(size_t longueur, Points<T> &p){
// Constructor initializing the Vec_Points with longueur copies of Points p
	m_pV = new vecpoint_t<T>{};
	for (size_t i{0}; i < longueur; ++i) {
		m_pV->push_back(p);
	}
}

template <typename T>
inline Vec_Points<T>::Vec_Points(size_t longueur) {
// Constructor initializing the Vec_Points with longueur copies of null points
	m_pV = new vecpoint_t<T>{};
	Points<T> zeros {0, 0, 0};
	for (size_t i{0}; i < longueur; ++i) {
		m_pV->push_back(zeros);
	}
}

template <typename T>
inline void Vec_Points<T>::push_back (const Points<T> &p){
	m_pV->push_back(p);
}

template <typename T>
inline void Vec_Points<T>::push_back(T x, T y, T z){
	Points<T> p1{x,y,z};
	push_back(p1);
}

template <typename T>
inline void Vec_Points<T>::pop_back(){
	m_pV->pop_back();
}

template <typename T>
bool Vec_Points<T>::load_vecpoints(std::string &path){
	std::ifstream inputFile{};
	std::string str(""), str1(""), str2(""), str3(""), str4("");
	std::string token;
	Points<T> point{};
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
bool Vec_Points<T>::save_vecpoints(std::string path) const {

	std::ofstream outputFile{};

	outputFile.open (path);
	if (outputFile.is_open()) {
		outputFile << "[";
		for (size_t i{0}; i<m_pV->size()-1; i++) {
			outputFile << "[" << to_string_with_precision((*m_pV)[i][0], 10) << ", " <<
					to_string_with_precision((*m_pV)[i][1], 10) << ", " <<
					to_string_with_precision((*m_pV)[i][2], 10) << "];\n";
		}
		outputFile << "[" << to_string_with_precision((*m_pV)[m_pV->size()-1][0], 10) << ", " <<
							to_string_with_precision((*m_pV)[m_pV->size()-1][1], 10) << ", " <<
							to_string_with_precision((*m_pV)[m_pV->size()-1][2], 10) << "]];\n";
		outputFile.close();
	} else {
		std::cerr << "Error: Unable to open the file \"" << path << "\"";
		return true;
	}

	return false;
}

template <typename T>
inline void Vec_Points<T>::assign (size_t longueur, Points<T> &p) {
// Assigns to the Vec_Points longueur copies of Points p

	if (m_pV->size() < longueur) {
		for (size_t i{0}; i< m_pV->size(); ++i) {
			m_pV->at(i) = p;
		}
		for (size_t i{m_pV->size()}; i < longueur; ++i){
			m_pV->push_back(p);
		}
	} else {
		for (size_t i{0}; i< longueur; ++i) {
			m_pV->at(i) = p;
		}
	}
}

template <typename T>
inline Points<T> Vec_Points<T>::mean() const{
	T x{0}, y{0}, z{0};
	for (auto p:(*m_pV)) {
		x+=p[0];
		y+=p[1];
		z+=p[2];
	}
	size_t length{m_pV->size()};
	Points<T> p{x/length, y/length, z/length};
	return p;
}

template <typename T>
inline const Points<T> & Vec_Points<T>::operator [](const size_t i) const{
	if (i < m_pV->size()) {
		return (*m_pV)[i];
	} else {
		throw std::out_of_range ("Invalid access to Vec_Points elements");
	}
}

template <typename T>
inline Points<T> & Vec_Points<T>::operator [](const size_t i){
	if (i < m_pV->size()) {
		return (*m_pV)[i];
	} else {
		throw std::out_of_range ("Invalid access to Vec_Points elements");
	}
}

template <typename T>
inline Vec_Points<T> & Vec_Points<T>::operator=(const Vec_Points<T> &a) {
	if (this != &a){
		*(this->m_pV)=*(a.m_pV);
	}
	return *this;
}

template <typename T>
inline Mat_33<T> Vec_Points<T>::operator*(const Vec_Points<T> &b) const {
// Multiplication of Vec_Points transposed by Vec_Points
	if (this->size() != b.size()) {
		throw std::runtime_error("The size of the vector of points transposed does not match to the size of the vector of points.");
	}
	T a00 { 0 }, a01 { 0 }, a02 { 0 }, a10 { 0 }, a11 { 0 }, a12 { 0 }, a20 { 0 }, a21 { 0 }, a22 { 0 };
	for (size_t i { 0 }; i < this->size(); ++i) {
		Points<T> x { (*m_pV)[i] };
		Points<T> y { (*(b.m_pV))[i] };
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
inline Vec_Points<T>  Vec_Points<T>::operator*(const std::vector<T> &p) const{
// Multiplication of Vec_Points with a vector of scalars
	if (p.size() != m_pV->size()) {
		throw std::runtime_error("The size of the vector of points does not match to the size of the vector.");
	}
	Vec_Points<T> temp{};
	Points<T> tempP{};
	auto itr = p.begin();
	for (const auto &x:(*m_pV)){
		tempP[0] = x[0] * (*itr);
		tempP[1] = x[1] * (*itr);
		tempP[2] = x[2] * (*itr++);
		temp.push_back(tempP);
	}
	return temp;
}

template <typename T>
inline Vec_Points<T> Vec_Points<T>::operator*(const Mat_33<T> &a) const{
// Multiplication of Vec_Points with a Mat_33
	Vec_Points<T> temp{};
	Points<T> tempP{};
	for (const auto &x:(*m_pV)){
		tempP[0] = x[0] * a[0][0] + x[1] * a[1][0] + x[2] * a[2][0];
		tempP[1] = x[0] * a[0][1] + x[1] * a[1][1] + x[2] * a[2][1];
		tempP[2] = x[0] * a[0][2] + x[1] * a[1][2] + x[2] * a[2][2];
		temp.push_back(tempP);
	}
	return temp;
}

template <typename T>
inline Vec_Points<T> Vec_Points<T>::operator-(const Points<T> &p) {
	Vec_Points<T> temp{};
	Points<T> temp_P{};
	for (const auto &x : (*m_pV)) {
		temp_P[0] = x[0] - p[0];
		temp_P[1] = x[1] - p[1];
		temp_P[2] = x[2] - p[2];
		temp.push_back(temp_P);
	}
	return temp;
}

template <typename T>
inline Vec_Points<T>::~Vec_Points() {
	delete m_pV;
}

#endif /* SRC_VEC_POINTS_HPP_ */
