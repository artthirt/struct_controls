#ifndef QUATERNIONS
#define QUATERNIONS

#include <QDebug>

#include "common_.h"
#include "vector3_.h"

namespace quaternions {

struct Quaternion;

static inline Quaternion operator- (const Quaternion& q1, const Quaternion q2);
static inline Quaternion operator+ (const Quaternion& q1, const Quaternion q2);
static inline Quaternion operator* (const Quaternion& q1, const Quaternion q2);
static inline Quaternion operator* (const Quaternion& q1, double t);

//////////////////////////////////////////////////
/// \brief The Quaternion struct
/// simple class for quaternion
struct Quaternion{
	vector3_::Vector3d v;
	double w;

	Quaternion(){
		w = 1;
	}
	Quaternion(const Quaternion& q){
		v = q.v;
		w = q.w;
	}
	Quaternion(double x, double y, double z, double r){
		v = vector3_::Vector3d(x, y, z);
		w = r;
	}
	Quaternion(const vector3_::Vector3d& vector, double r){
		w = r;
		v = vector;
	}
	bool isNull() const{
		return w == 1. && v.x() == 0. && v.y() == 0. && v.z() == 0.;
	}
	inline double x() const{
		return v.x();
	}
	inline double y() const{
		return v.y();
	}
	inline double z() const{
		return v.z();
	}
	inline double length() const{
		double len = v.x() * v.x() + v.y() * v.y() +
				v.z() * v.z() + w * w;
		return sqrt(len);
	}
	inline double lengthSquared() const{
		double len = v.x() * v.x() + v.y() * v.y() +
				v.z() * v.z() + w * w;
		return len;
	}
	Quaternion conj() const{
		return Quaternion(v.inv(), w);
	}
	void normalize(){
		double len = v.x() * v.x() + v.y() * v.y() +
				v.z() * v.z() + w * w;
		if(common_::fIsNull(len) || common_::fIsNull(len - 1.0))
			return;

		len = 1.0/sqrt(len);
		v *= len;
		w *= len;
	}
	Quaternion normalized() const{
		Quaternion res(*this);
		res.normalize();
		return res;
	}
	vector3_::Vector3d rotatedVector(const vector3_::Vector3d& val) const{
		Quaternion res = *this * Quaternion(val, 0) * conj();
		return res.v;
	}
	Quaternion& operator= (const Quaternion& q){
		v = q.v;
		w = q.w;
		return *this;
	}
	Quaternion& operator *= (const Quaternion& q){
		*this = *this * q;
		return *this;
	}
	Quaternion& operator *= (double value){
		w *= value;
		v *= value;
		return *this;
	}
	Quaternion& operator+= (const Quaternion& q){
		w += q.w;
		v += q.v;
		return *this;
	}
	Quaternion& operator-= (const Quaternion& q){
		w -= q.w;
		v -= q.v;
		return *this;
	}

	static Quaternion fromAxisAndAngle(double x, double y, double z, double angle){
		Quaternion q;
		double a = common_::angle2rad(angle/2.0);
		q.w = cos(a);
		double s = sin(a);
		q.v = vector3_::Vector3d(x, y, z) * s;
		q.normalize();
		return q;
	}
	static Quaternion fromAxisAndAngle(const vector3_::Vector3d& axis, double angle){
		Quaternion q;
		double a = common_::angle2rad(angle/2.0);
		q.w = cos(a);
		double s = sin(a);
		q.v = axis * s;
		q.normalize();
		return q;
	}
	static double dot(const Quaternion& q1, const Quaternion& q2){
		double d = vector3_::Vector3d::dot(q1.v, q2.v);
		return d + q1.w * q2.w;
	}
	static Quaternion nlerp(const Quaternion& p0, const Quaternion& p1, double t){
		if(t <= 0)
			return p0;
		if(t >= 1)
			return p1;
		Quaternion res = p0 * (1 - t) + p1 * t;
		return res.normalized();
	}
	static Quaternion slerp(const Quaternion& p0, const Quaternion& p1, double t){
		Quaternion res;
		double dot = Quaternion::dot(p0, p1);

		if(t <= 0)
			return p0;
		if(t >= 1)
			return p1;
		if(common_::fIsNull(dot))
			return p0;

		double f1 = 1 - t;
		double f2 = t;

		if((1 - dot) > 0.0000001){
			double theta = acos(dot);
			double sinTheta = sin(theta);
			if(sinTheta > 0.0000001){
				f1 = sin((1 - t) * theta) / sinTheta;
				f2 = sin(t * theta) / sinTheta;
			}
		}
		res = p0 * f1 + p1 * f2;
		return res.normalized();
	}
};

//////////////////////////////////////////////////

static inline Quaternion operator* (const Quaternion& q1, const Quaternion q2)
{
	// q1(a, b, c, d) q2(e, f, g, h)
	// (ae-bf-cg-dh)+(af+be+ch-dg)i+(ag-bh+ce+df)j+(ah+bg-cf+de)k

	Quaternion res;
	double vx, vy, vz;

	res.w = q1.w * q2.w - q1.v.x() * q2.v.x() - q1.v.y() * q2.v.y() - q1.v.z() * q2.v.z();

	vx = q1.w * q2.v.x() + q1.v.x() * q2.w + q1.v.y() * q2.v.z() - q1.v.z() * q2.v.y();
	vy = q1.w * q2.v.y() - q1.v.x() * q2.v.z() + q1.v.y() * q2.w + q1.v.z() * q2.v.x();
	vz = q1.w * q2.z() + q1.v.x() * q2.v.y() - q1.v.y() * q2.v.x() + q1.v.z() * q2.w;

	res.v = vector3_::Vector3d(vx, vy, vz);

	return res;
}

static inline Quaternion operator* (const Quaternion& q1, double t)
{
	return Quaternion(q1.v * t, q1.w * t);
}

static inline Quaternion operator+ (const Quaternion& q1, const Quaternion q2)
{
	return Quaternion(q1.v + q2.v, q1.w + q2.w);
}

static inline Quaternion operator- (const Quaternion& q1, const Quaternion q2)
{
	return Quaternion(q1.v - q2.v, q1.w - q2.w);
}

static inline QDebug operator<< (QDebug dbg, const Quaternion& q)
{
	dbg.nospace() << "(" << q.w << " [" << q.v.x() << ", " << q.v.y() << ", " << q.v.z() << "] )";
	return dbg.space();
}

}

#endif // QUATERNIONS

