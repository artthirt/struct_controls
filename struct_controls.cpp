#include "struct_controls.h"

using namespace sc;

StructServo::StructServo()
{
	freq_meandr = 0;
	angle = 0;
	speed_of_change = 0;
	timework_ms = 0;
	flag_start = false;
	pin = 0;
}

void StructServo::write_to(QDataStream &stream)
{
	stream << freq_meandr;
	stream << angle;
	stream << speed_of_change;
	stream << timework_ms;
	stream << flag_start;
	stream << pin;
}

void StructServo::read_from(QDataStream &stream)
{
	stream >> freq_meandr;
	stream >> angle;
	stream >> speed_of_change;
	stream >> timework_ms;
	stream >> flag_start;
	stream >> pin;
}

bool StructServo::trigger_start(const StructServo &last) const
{
	return flag_start && !last.flag_start;
}

////////////////////////////////////////////////////

StructControls::StructControls()
{
	power_on = false;
	throttle = 0;
	tangaj = 0;
	bank = 0;
	yaw = 0;
}

void StructControls::write_to(QDataStream &stream)
{
	stream.setByteOrder(QDataStream::BigEndian);
	stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
	stream.setVersion(QDataStream::Qt_4_8);

	stream << power_on;
	stream << throttle;
	stream << tangaj;
	stream << bank;
	stream << yaw;
	servo_ctrl.write_to(stream);
}

void StructControls::read_from(QDataStream &stream)
{
	stream.setByteOrder(QDataStream::BigEndian);
	stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
	stream.setVersion(QDataStream::Qt_4_8);

	stream >> power_on;
	stream >> throttle;
	stream >> tangaj;
	stream >> bank;
	stream >> yaw;
	servo_ctrl.read_from(stream);
}

////////////////////////////////////////////////////

StructGyroscope::StructGyroscope()
{
	fs_sel = 0;
	afs_sel = 0;
	freq = default_freq;
	tick = 0;
	temp = 0;
	FOREACH(i, raw_count, raw[i] = 0);
}

StructGyroscope::StructGyroscope(const StructGyroscope& st)
{
	gyro = st.gyro;
	accel = st.accel;
	afs_sel = st.afs_sel;
	fs_sel = st.fs_sel;
	freq = st.freq;
	tick = st.tick;
	std::copy(st.raw, st.raw + raw_count, raw);
	temp = st.temp;
}

void StructGyroscope::write_to(QDataStream& stream)
{
	stream << temp;
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
void StructGyroscope::read_from(QDataStream& stream)
{
	stream >> temp;
	FOREACH(i, vector3_::Vector3i::count, stream >> gyro[i]);
	FOREACH(i, vector3_::Vector3i::count, stream >> accel[i]);
	stream >> afs_sel;
	stream >> fs_sel;
	stream >> freq;
	stream >> tick;
	stream.readRawData(reinterpret_cast< char* >(raw), raw_count);
}

vector3_::Vector3d StructGyroscope::angular_speed(const vector3_::Vector3d& offset)
{
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

////////////////////////////////////////////////

StructCompass::StructCompass()
{
	mode = tick = 0;
}

void StructCompass::read_from(QDataStream &stream)
{
	stream >> mode;
	stream >> tick;
	stream >> data.x();
	stream >> data.y();
	stream >> data.z();
}

void StructCompass::write_to(QDataStream &stream)
{
	stream << mode;
	stream << tick;
	stream << data.x();
	stream << data.y();
	stream << data.z();
}

////////////////////////////////////////////////

StructBarometer::StructBarometer()
{
	tick = data = temp = 0;
}

void StructBarometer::write_to(QDataStream &stream)
{
	stream << tick;
	stream << data;
	stream << temp;
}

void StructBarometer::read_from(QDataStream &stream)
{
	stream >> tick;
	stream >> data;
	stream >> temp;
}

////////////////////////////////////////////////
////////////////////////////////////////////////

/**
 * @brief StructTelemetry
 */
StructTelemetry::StructTelemetry()
{
	FOREACH(i, cnt_engines, power[i] = 0;);
	power_on = false;
	height = 0;
	course = tangaj = bank = 0;
}

/**
 * @brief StructTelemetry
 * @param st
 */
StructTelemetry::StructTelemetry(const StructTelemetry& st)
{
	power_on = st.power_on;
	FOREACH(i, cnt_engines, power[i] = st.power[i];);
	tangaj = st.tangaj;
	bank = st.bank;
	course = st.course;
	height = st.height;
	gyroscope = st.gyroscope;
}

/**
 * @brief write_to
 * serialize to byte array
 * @param stream
 */
void StructTelemetry::write_to(QDataStream& stream)
{
	stream.setByteOrder(QDataStream::BigEndian);
	stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
	stream.setVersion(QDataStream::Qt_4_8);

	stream << power_on;
	FOREACH(i, cnt_engines, stream << power[i]);
	stream << tangaj;
	stream << bank;
	stream << course;
	stream << height;
	gyroscope.write_to(stream);
	compass.write_to(stream);
	barometer.write_to(stream);
}

/**
 * @brief read_from
 * deserialize byte array
 * @param stream
 */
void StructTelemetry::read_from(QDataStream& stream)
{
	stream.setByteOrder(QDataStream::BigEndian);
	stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
	stream.setVersion(QDataStream::Qt_4_8);

	stream >> power_on;
	FOREACH(i, cnt_engines, stream >> power[i]);
	stream >> tangaj;
	stream >> bank;
	stream >> course;
	stream >> height;
	gyroscope.read_from(stream);
	compass.read_from(stream);
	barometer.read_from(stream);
}

