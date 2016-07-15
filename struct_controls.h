#ifndef STRUCT_CONTROLS_H
#define STRUCT_CONTROLS_H

#include <vector>
#include <math.h>

#ifndef WITHOUT_QT
#include <QDataStream>
#include <QDebug>
#else
#include "iostream"
#endif

#include "common_.h"
#include "vector3_.h"

#ifdef WITHOUT_QT
#include "datastream.h"
#define QDataStream datastream
#else
#endif

namespace sc{

struct StructAngleCtrl{
	StructAngleCtrl(){
		pin = 1;
		angle = 0;
		freq = 500;
		timework_ms = 10;
	}

	int pin;
	int timework_ms;
	float angle;
	float freq;
};

struct StructServo{
	StructServo();

	void write_to(QDataStream& stream);
	void read_from(QDataStream& stream);
	bool trigger_start(const StructServo& last) const;

	float freq_meandr;
	float angle;
	float speed_of_change;
	float timework_ms;

	bool flag_start;
	int pin;
};

struct StructControls
{
	StructControls();


	void write_to(QDataStream& stream);
	void read_from(QDataStream& stream);

	bool power_on;
	float throttle;
	float tangaj;
	float bank;
	float yaw;
	StructServo servo_ctrl;
};

const int cnt_engines = 4;
const int raw_count = 46;
const float default_freq = 100;

struct StructGyroscope{

	StructGyroscope();
	StructGyroscope(const StructGyroscope& st);

	void write_to(QDataStream& stream);
	/**
	 * @brief read_from
	 * deserialize byte array
	 * @param stream
	 */
	void read_from(QDataStream& stream);

	/**
	 * @brief angular_speed
	 * get angular speed from gyroscope mpu6050
	 * uses fs_sel for get factor to get the real angles
	 * @param offset
	 * @return
	 */
	vector3_::Vector3d angular_speed(const vector3_::Vector3d& offset = vector3_::Vector3d());

	vector3_::Vector3i gyro;
	vector3_::Vector3i accel;

	unsigned char afs_sel;				/// value of accelerometer mode
	unsigned char fs_sel;				/// value of gyroscope mode
	float temp;
	float freq;							/// It is used to define part of the data
										/// from the sensors to a single point in time
	long long tick;
	unsigned char raw[raw_count];		/// raw data from 0x0d to 0x3a address from mpu6050

};

struct StructCompass{
	StructCompass();

	void read_from(QDataStream& stream);
	void write_to(QDataStream& stream);

	long long tick;
	unsigned char mode;
	vector3_::Vector3i data;
};

struct StructBarometer{
	StructBarometer();

	void write_to(QDataStream& stream);
	void read_from(QDataStream& stream);

	int data;
	int temp;
	long long tick;
};

/**
 * @brief The StructTelemetry struct
 * the structure for work with mpu6050 with raspberry pi 2 in the project "example_rpi2"
 */
struct StructTelemetry
{
	/**
	 * @brief StructTelemetry
	 */
	StructTelemetry();
	/**
	 * @brief StructTelemetry
	 * @param st
	 */
	StructTelemetry(const StructTelemetry& st);
	/**
	 * @brief write_to
	 * serialize to byte array
	 * @param stream
	 */
	void write_to(QDataStream& stream);
	/**
	 * @brief read_from
	 * deserialize byte array
	 * @param stream
	 */
	void read_from(QDataStream& stream);

	bool power_on;

	float power[cnt_engines];

	float tangaj;
	float bank;
	float course;
	float height;

	StructGyroscope gyroscope;
	StructCompass compass;
	StructBarometer barometer;

};

}

#endif // STRUCT_CONTROLS_H
