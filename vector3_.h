#ifndef VECTOR3_
#define VECTOR3_

#include "common_.h"

namespace vector3_{

////////////////////////////////////////////////////

template< typename T >
struct Vector3_{
	enum{
		count = 3
	};
	Vector3_(){
		FOREACH(i, count, data[i] = 0);
	}
	Vector3_(T x, T y, T z){
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}
	Vector3_(const Vector3_& v){
		FOREACH(i, count, data[i] = v.data[i];);
	}
	template< typename P >
	Vector3_(const Vector3_<P> &v)
	{
		FOREACH(i, count, data[i] = static_cast< T > (v.data[i]));
	}

	inline const T& x() const { return data[0]; }
	inline const T& y() const { return data[1]; }
	inline const T& z() const { return data[2]; }
	inline T& x() { return data[0]; }
	inline T& y() { return data[1]; }
	inline T& z() { return data[2]; }
	inline void setX(T value) { data[0] = value; }
	inline void setY(T value) { data[1] = value; }
	inline void setZ(T value) { data[2] = value; }

	inline bool isNull() const{
		T res = 0;
		FOREACH(i, count, res += data[i] * data[i]);
		return res < common_::epsilon;
	}
	inline Vector3_& operator*= (T value){
		FOREACH(i, count, data[i] *= value);
		return *this;
	}
	inline Vector3_& operator+= (const Vector3_& v){
		FOREACH(i, count, data[i] += v.data[i]);
		return *this;
	}
	inline Vector3_& operator-= (const Vector3_& v){
		FOREACH(i, count, data[i] -= v.data[i]);
		return *this;
	}
	inline T& operator[] (int index){
		ASSERT_EC(index >=0 && index < count, "index out of range");
		return data[index];
	}
	inline void clear(){
		FOREACH(i, count, data[i] = 0);
	}
	inline T& operator[] (int index) const{
		ASSERT_EC(index >=0 && index < count, "index out of range");
		return data[index];
	}
	inline double length() const{
		double res = 0;
		FOREACH(i, count, res += data[i] * data[i]);
		return sqrt(res);
	}
	inline double length_square() const{
		double res = 0;
		FOREACH(i, count, res += data[i] * data[i]);
		return res;
	}
	inline Vector3_ normalize(){
		double res = length();
		double rabs = fabs(res);
		if(rabs < 1e-7)
			return *this;
		res = 1.0 / res;
		FOREACH(i, count, data[i] *= res);
		return *this;
	}
	inline Vector3_ normalized() const{
		Vector3_ res(*this);
		res.normalize();
		return res;
	}
	inline Vector3_ inv() const{
		return Vector3_(-x(), -y(), -z());
	}
	operator QString() const{
		return QString("[%1; %2; %3]").arg(x()).arg(y()).arg(z());
	}

	static double dot(const Vector3_& v1, const Vector3_& v2){
		double res = 0;
		FOREACH(i, count, res += v1.data[i] * v2.data[i]);
		return res;
	}
	static Vector3_ cross(const Vector3_& v1, const Vector3_& v2){
		Vector3_ res;
		res.setX(v1.y() * v2.z() - v1.z() * v2.y());
		res.setY(v1.z() * v2.x() - v1.x() * v2.z());
		res.setZ(v1.x() * v2.y() - v1.y() * v2.x());
		return res;
	}

	T data[count];
};


/**
 * @brief operator +
 * @param v1
 * @param v2
 * @return
 */
template< typename T >
static inline Vector3_< T > operator+ (const Vector3_< T >& v1, const Vector3_< T >& v2){
	Vector3_< T > res;
	FOREACH(i, Vector3_< T >::count, res.data[i] = v1.data[i] + v2.data[i]);
	return res;
}

/**
 * @brief operator -
 * @param v1
 * @param v2
 * @return
 */
template< typename T >
static inline Vector3_< T > operator- (const Vector3_< T >& v1, const Vector3_< T >& v2){
	Vector3_< T > res;
	FOREACH(i, Vector3_< T >::count, res.data[i] = v1.data[i] - v2.data[i]);
	return res;
}

/**
 * @brief operator *
 * multiple vector to single value
 * @param v
 * @param d
 * @return
 */
template< typename T >
static inline Vector3_< T > operator* (const Vector3_< T >& v, T d){
	Vector3_< T > res;
	FOREACH(i, Vector3_< T >::count, res.data[i] = v.data[i] * d);
	return res;
}

/**
 * @brief operator *
 * multiple vector to single value
 * @param v1
 * @param v2
 * @return
 */
template< typename T >
static inline Vector3_< T > operator* (const Vector3_< T >& v1, const Vector3_< T >& v2){
	Vector3_< T > res;
	FOREACH(i, Vector3_< T >::count, res.data[i] = v1.data[i] * v2.data[i]);
	return res;
}


typedef Vector3_< float > Vector3f;
typedef Vector3_< double > Vector3d;
typedef Vector3_< int > Vector3i;

}

#endif // VECTOR3_

