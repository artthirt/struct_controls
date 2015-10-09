#ifndef STRUCT_CONTROLS_H
#define STRUCT_CONTROLS_H

#include <vector>
#include <math.h>
#include <QDataStream>
#include <QDebug>

namespace sc{

/// \brief FOREACH - cycle for the expression at
/// the specified index and the number of repetitions
#define FOREACH(index, cnt, expression) for(int index = 0; index < cnt; index++){ \
	expression; \
	}

///////////////////////////////////////////////

struct Quaternion;

static inline Quaternion operator- (const Quaternion& q1, const Quaternion q2);
static inline Quaternion operator+ (const Quaternion& q1, const Quaternion q2);
static inline Quaternion operator* (const Quaternion& q1, const Quaternion q2);
static inline Quaternion operator* (const Quaternion& q1, double t);

///////////////////////////////////////////////
/// \brief The ExceptionCustom class
/// simple class for exception
class ExceptionCustom{
public:
	enum{
		UNKNOWN = 0
	};
	std::string message;
	int code;

	ExceptionCustom(std::string msg){
		message = msg;
		code = UNKNOWN;
	}
	ExceptionCustom(int code){
		this->code = code;
	}
};

#ifdef _DEBUG
#define ASSERT_EC(val, msg)	if(!(val)) throw new ExceptionCustom(msg)
#else
#define ASSERT_EC(val, msg)
#endif

const double epsilon = 1e-9;

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

	inline T x() const { return data[0]; }
	inline T y() const { return data[1]; }
	inline T z() const { return data[2]; }
	inline void setX(T value) { data[0] = value; }
	inline void setY(T value) { data[1] = value; }
	inline void setZ(T value) { data[2] = value; }

	inline bool isNull() const{
		T res = 0;
		FOREACH(i, count, res += data[i] * data[i]);
		return res < epsilon;
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

typedef Vector3_< float > Vector3f;
typedef Vector3_< double > Vector3d;
typedef Vector3_< int > Vector3i;

//////////////////////////////////////////////////

/**
 * @brief fIsNull
 * compare floating value for null
 * @param value
 * @return
 */
static inline bool fIsNull(double value)
{
	return fabs(value) < epsilon;
}

/**
 * @brief angle2rad
 * convert angle to radian
 * @param angle
 * @return
 */
static inline double angle2rad(double angle)
{
	return angle * M_PI / 180.0;
}

/**
 * @brief rad2angle
 * convert radian to angle
 * @param rad
 * @return
 */
static inline double rad2angle(double rad)
{
	return rad * 180.0 / M_PI;
}

//////////////////////////////////////////////////
/// \brief The Quaternion struct
/// simple class for quaternion
struct Quaternion{
	Vector3d v;
	double w;

	Quaternion(){
		w = 1;
	}
	Quaternion(const Quaternion& q){
		v = q.v;
		w = q.w;
	}
	Quaternion(double x, double y, double z, double r){
		v = Vector3d(x, y, z);
		w = r;
	}
	Quaternion(const Vector3d& vector, double r){
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
		if(fIsNull(len) || fIsNull(len - 1.0))
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
	Vector3d rotatedVector(const Vector3d& val) const{
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
		double a = angle2rad(angle/2.0);
		q.w = cos(a);
		double s = sin(a);
		q.v = Vector3d(x, y, z) * s;
		q.normalize();
		return q;
	}
	static Quaternion fromAxisAndAngle(const Vector3d& axis, double angle){
		Quaternion q;
		double a = angle2rad(angle/2.0);
		q.w = cos(a);
		double s = sin(a);
		q.v = axis * s;
		q.normalize();
		return q;
	}
	static double dot(const Quaternion& q1, const Quaternion& q2){
		double d = Vector3d::dot(q1.v, q2.v);
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
		if(fIsNull(dot))
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

	res.v = Vector3d(vx, vy, vz);

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

//////////////////////////////////////////////////

struct StructControls
{
	bool power_on;
	float throttle;
	float tangaj;
	float bank;
	float rotation;
};

const int cnt_engines = 4;
const int raw_count = 46;
const float default_freq = 100;

/**
 * @brief The StructTelemetry struct
 * the structure for work with mpu6050 with raspberry pi 2 in the project "example_rpi2"
 */
struct StructTelemetry
{
	/**
	 * @brief StructTelemetry
	 */
	StructTelemetry(){
		FOREACH(i, cnt_engines, power[i] = 0;);
		power_on = false;
		height = 0;
		temp = 0;
		course = tangaj = bank = 0;
		fs_sel = 0;
		afs_sel = 0;
		freq = default_freq;
		tick = 0;
		FOREACH(i, raw_count, raw[i] = 0);
	}
	/**
	 * @brief StructTelemetry
	 * @param st
	 */
	StructTelemetry(const StructTelemetry& st){
		power_on = st.power_on;
		FOREACH(i, cnt_engines, power[i] = st.power[i];);
		tangaj = st.tangaj;
		bank = st.bank;
		course = st.course;
		gyro = st.gyro;
		accel = st.accel;
		height = st.height;
		temp = st.temp;
		afs_sel = st.afs_sel;
		fs_sel = st.fs_sel;
		freq = st.freq;
		tick = st.tick;
		std::copy(st.raw, st.raw + raw_count, raw);
	}
	/**
	 * @brief write_to
	 * serialize to byte array
	 * @param stream
	 */
	void write_to(QDataStream& stream){
		stream.setByteOrder(QDataStream::BigEndian);
		stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
		stream.setVersion(QDataStream::Qt_4_8);

		stream << power_on;
		FOREACH(i, cnt_engines, stream << power[i]);
		stream << tangaj;
		stream << bank;
		stream << course;
		stream << temp;
		stream << height;
		FOREACH(i, Vector3i::count, stream << gyro[i]);
		FOREACH(i, Vector3i::count, stream << accel[i]);
		stream << afs_sel;
		stream << fs_sel;
		stream << freq;
		stream << tick;
		stream.writeRawData(reinterpret_cast< char* >(raw), raw_count);
	}
	/**
	 * @brief read_from
	 * deserialize byte array
	 * @param stream
	 */
	void read_from(QDataStream& stream){
		stream.setByteOrder(QDataStream::BigEndian);
		stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
		stream.setVersion(QDataStream::Qt_4_8);

		stream >> power_on;
		FOREACH(i, cnt_engines, stream >> power[i]);
		stream >> tangaj;
		stream >> bank;
		stream >> course;
		stream >> temp;
		stream >> height;
		FOREACH(i, Vector3i::count, stream >> gyro[i]);
		FOREACH(i, Vector3i::count, stream >> accel[i]);
		stream >> afs_sel;
		stream >> fs_sel;
		stream >> freq;
		stream >> tick;
		stream.readRawData(reinterpret_cast< char* >(raw), raw_count);
	}

	/**
	 * @brief angular_speed
	 * get angular speed from gyroscope mpu6050
	 * uses fs_sel for get factor to get the real angles
	 * @param offset
	 * @return
	 */
	Vector3d angular_speed(const Vector3d& offset = Vector3d()){
		float factor = 1.0;
		Vector3d g = gyro;
		g -= offset;
		switch (fs_sel) {
			case 0:
			default:
				factor = 250.0f / 32768.0f;
				break;
			case 1:
				factor = 500.0f / 32768.0f;
				break;
			case 2:
				factor = 1000.0f / 32768.0f;
				break;
			case 3:
				factor = 2000.0f / 32768.0f;
				break;
		}
		Vector3d res = Vector3d(g);
		if(!freq)
			freq = default_freq;

		res *= factor;
		return res;
	}

	bool power_on;

	float power[cnt_engines];

	float tangaj;
	float bank;
	float course;
	float temp;
	float height;

	Vector3i gyro;

	Vector3i accel;

	unsigned char afs_sel;				/// value of accelerometer mode
	unsigned char fs_sel;				/// value of gyroscope mode
	float freq;							/// It is used to define part of the data
										/// from the sensors to a single point in time
	long long tick;
	unsigned char raw[raw_count];		/// raw data from 0x0d to 0x3a address from mpu6050
};

}

#endif // STRUCT_CONTROLS_H
