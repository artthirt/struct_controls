#ifndef STRUCT_CONTROLS_H
#define STRUCT_CONTROLS_H

#include <vector>
#include <math.h>
#include <QDataStream>
#include <QDebug>

#include "common_.h"
#include "vector3_.h"

namespace sc{

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
		FOREACH(i, vector3_::Vector3i::count, stream << gyro[i]);
		FOREACH(i, vector3_::Vector3i::count, stream << accel[i]);
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
		FOREACH(i, vector3_::Vector3i::count, stream >> gyro[i]);
		FOREACH(i, vector3_::Vector3i::count, stream >> accel[i]);
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
	vector3_::Vector3d angular_speed(const vector3_::Vector3d& offset = vector3_::Vector3d()){
		float factor = 1.0;
		vector3_::Vector3d g = gyro;
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
		vector3_::Vector3d res = vector3_::Vector3d(g);
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

	vector3_::Vector3i gyro;

	vector3_::Vector3i accel;

	unsigned char afs_sel;				/// value of accelerometer mode
	unsigned char fs_sel;				/// value of gyroscope mode
	float freq;							/// It is used to define part of the data
										/// from the sensors to a single point in time
	long long tick;
	unsigned char raw[raw_count];		/// raw data from 0x0d to 0x3a address from mpu6050
};

}

#endif // STRUCT_CONTROLS_H
