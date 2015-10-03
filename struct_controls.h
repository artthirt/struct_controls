#ifndef STRUCT_CONTROLS_H
#define STRUCT_CONTROLS_H

#include <vector>
#include <math.h>
#include <QDataStream>

/// \brief FOREACH - cycle for the expression at
/// the specified index and the number of repetitions
#define FOREACH(index, cnt, expression) for(int index = 0; index < cnt; index++){ \
	expression; \
	}

///////////////////////////////////////////////

class Quaternion;

static inline Quaternion operator- (const Quaternion& q1, const Quaternion q2);
static inline Quaternion operator+ (const Quaternion& q1, const Quaternion q2);
static inline Quaternion operator* (const Quaternion& q1, const Quaternion q2);

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
 * @brief isNull
 * compare value for null
 * @param value
 * @return
 */
static inline bool isNull(double value)
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
	inline double x(){
		return v.x();
	}
	inline double y(){
		return v.y();
	}
	inline double z(){
		return v.z();
	}
	Quaternion conj() const{
		return Quaternion(v.inv(), w);
	}
	void normalize(){
		double len = v.x() * v.x() + v.y() * v.y() +
				v.z() * v.z() + w * w;
		if(isNull(len) || isNull(len - 1.0f));
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
	}

	static Quaternion fromAxisAngle(double x, double y, double z, double angle){
		Quaternion q;
		double a = angle2rad(angle/2.0);
		q.w = cos(a);
		double s = sin(a);
		q.v = Vector3d(x, y, z).normalized();
		q.v *= s;
		return q;
	}
	static Quaternion fromAxisAngle(const Vector3d& axis, double angle){
		Quaternion q;
		double a = angle2rad(angle/2.0);
		q.w = cos(a);
		double s = sin(a);
		q.v = axis.normalized();
		q.v *= s;
		return q;
	}
};

//////////////////////////////////////////////////

static inline Quaternion operator* (const Quaternion& q1, const Quaternion q2)
{
/*
 *	{
 *		result.w:=q1.w*q2.w-DotProduct(q1.v,q2.v);
 *		result.v:=AddVertex(AddVertex(MULUVertex(q1.w,q2.v),MULUVertex(q2.w,q1.v)),MultVertex(q1.v,q2.v));
 *	}
*/
	Quaternion res;
	res.w = q1.w * q2.w - Vector3d::dot(q1.v, q2.v);
	Vector3d v1 = q1.v * q2.w;
	Vector3d v2 = q2.v * q1.w;
	Vector3d v3 = Vector3d::cross(q1.v, q2.v);
	res.v = v1 + v2 + v3;

	return res;
}

static inline Quaternion operator+ (const Quaternion& q1, const Quaternion q2)
{
	return Quaternion(q1.v + q2.v, q1.w + q2.w);
}

static inline Quaternion operator- (const Quaternion& q1, const Quaternion q2)
{
	return Quaternion(q1.v - q2.v, q1.w - q2.w);
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

#endif // STRUCT_CONTROLS_H
