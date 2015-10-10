#ifndef COMMON_
#define COMMON_

#include <math.h>

/// \brief FOREACH - cycle for the expression at
/// the specified index and the number of repetitions
#define FOREACH(index, cnt, expression) for(int index = 0; index < cnt; index++){ \
	expression; \
	}

namespace common_{

const double epsilon = 1e-9;

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

#ifdef DEBUG
#define ASSERT_EC(val, msg)	if(!(val)) throw new common_::ExceptionCustom(msg)
#else
#define ASSERT_EC(val, msg)
#endif

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

}

#endif // COMMON_

