#ifndef SRC_POINTS_HPP_
#define SRC_POINTS_HPP_

#include <array>
#include "Mat_33.hpp"

template <typename T>
class Points {
private:
	std::array <T, 3> *m_pA;
public:
	Points ();
	Points (T x, T y, T z);
	Points (const Points &obj);
	Points (Points &&obj);
	void SetValue (T x, T y, T z);
	T GetValue (size_t pos);
	virtual ~Points();
	Points<T> operator+(const Points<T> &a) const;
	Points<T> operator-(const Points<T> &a) const;
	Points<T> & operator=(const Points<T> &a);
	Points<T> & operator=(Points<T> &&a);
	T operator[](const int i) const;
	friend std::ostream & operator <<(std::ostream & out, const Points<T> &a) {
		out << "(" << a.m_pA->at(0) << ", " << a.m_pA->at(1) << ", " << a.m_pA->at(2) << ")" << std::endl;
		return out;
	}
};

template <typename T>
Points<T>::Points () {
	m_pA = new std::array <T, 3> {{0, 0, 0}};
};

template <typename T>
Points<T>::Points (T x, T y, T z) {
	m_pA = new std::array <T, 3> {{x, y, z}};
};

template <typename T>
Points<T>::Points (const Points &obj){
	m_pA = new std::array <T, 3> {{obj.m_pA->at(0), obj.m_pA->at(1), obj.m_pA->at(2)}};
}

template <typename T>
Points<T>::Points (Points &&obj) {
	m_pA = obj.m_pA;
	obj.m_pA = nullptr;
}

template <typename T>
void Points<T>::SetValue (T x, T y, T z) {
	m_pA->at(0) = x;
	m_pA->at(1) = y;
	m_pA->at(2) = z;
}

template <typename T>
T Points<T>::GetValue (size_t pos){
	if (pos>2) {
		throw std::runtime_error("Point coordinate must be between 0 and 2.");
	}
	return m_pA->at(pos);
}


template <typename T>
Points<T>::~Points() {
	delete m_pA;
}

template <typename T>
Points<T> & Points<T>::operator=(const Points<T> &a){
	if (this != &a) {
		delete m_pA;
		m_pA = new std::array <T, 3> {{a.m_pA->at(0), a.m_pA->at(1), a.m_pA->at(2)}};
	}
	return *this;
}

template <typename T>
Points<T> & Points<T>::operator=(Points<T> &&a){
	if (this != &a) {
		delete m_pA;
		m_pA = a.m_pA;
		a.m_pA = nullptr;
	}
	return *this;
}

template <typename T>
Points<T> Points<T>::operator+(const Points<T> &a) const{
	Points<T> temp{m_pA->at(0) + a.m_pA->at(0), m_pA->at(1) + a.m_pA->at(1), m_pA->at(2) + a.m_pA->at(2)};
	return temp;
}

template <typename T>
Points<T> Points<T>::operator-(const Points<T> &a) const{
	Points<T> temp{m_pA->at(0) - a.m_pA->at(0), m_pA->at(1) - a.m_pA->at(1), m_pA->at(2) - a.m_pA->at(2)};
	return temp;
}

template <typename T>
inline T Points<T>::operator[](const int i) const{
	return m_pA->at(i);
}



#endif /* SRC_POINTS_HPP_ */
