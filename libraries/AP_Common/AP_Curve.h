// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_Curve.h
/// @brief	used to transforms a pwm value to account for the non-linear pwm->thrust values of normal ESC+motors

#ifndef AP_CURVE
#define AP_CURVE

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library

/// @class      AP_Curve
template <class T, uint8_t SIZE>
class AP_Curve {
public:

    // Constructor
    AP_Curve();

    // clear - removes all points from the curve
    virtual void clear();

    // add_point - adds a point to the curve.  returns TRUE if successfully added
    virtual bool add_point( T x, T y );

    // get_y - returns the point on the curve at the given pwm_value (i.e. the new modified pwm_value)
    virtual T get_y( T x );

    // displays the contents of the curve (for debugging)
    virtual void dump_curve();

protected:
    uint8_t     _num_points;						// number of points in the cruve
    T           _x[SIZE];			// x values of each point on the curve
    T           _y[SIZE];			// y values of each point on the curve
    float       _slope[SIZE];		// slope between any two points.  i.e. slope[0] is the slope between points 0 and 1
    bool        _constrained;       // if true, first and last points added will constrain the y values returned by get_y function
};

// Typedef for convenience
typedef AP_Curve<int16_t,3> AP_CurveInt16_Size3;
typedef AP_Curve<int16_t,4> AP_CurveInt16_Size4;
typedef AP_Curve<int16_t,5> AP_CurveInt16_Size5;

typedef AP_Curve<uint16_t,3> AP_CurveUInt16_Size3;
typedef AP_Curve<uint16_t,4> AP_CurveUInt16_Size4;
typedef AP_Curve<uint16_t,5> AP_CurveUInt16_Size5;

// Constructor
template <class T, uint8_t SIZE>
AP_Curve<T,SIZE>::AP_Curve() :
	_num_points(0)
{
	// clear the curve
	clear();
};

// clear the curve
template <class T, uint8_t SIZE>
void AP_Curve<T,SIZE>::clear() {
	// clear the curve
	for( uint8_t i=0; i<SIZE; i++ ) {
		_x[i] = 0;
		_y[i] = 0;
		_slope[i] = 0.0;
	}
	_num_points = 0;
}

// add_point - adds a point to the curve
template <class T, uint8_t SIZE>
bool AP_Curve<T,SIZE>::add_point( T x, T y )
{
	if( _num_points < SIZE ) {
		_x[_num_points] = x;
		_y[_num_points] = y;

		// increment the number of points
		_num_points++;

		// if we have at least two points calculate the slope
		if( _num_points > 1 ) {
			_slope[_num_points-2] = (float)(_y[_num_points-1] - _y[_num_points-2]) / (float)(_x[_num_points-1] - _x[_num_points-2]);
			_slope[_num_points-1] = _slope[_num_points-2];	// the final slope is for interpolation beyond the end of the curve
		}
		return true;
	}else{
		// we do not have room for the new point
		return false;
	}
}

// get_y - returns the y value on the curve for a given x value
template <class T, uint8_t SIZE>
T AP_Curve<T,SIZE>::get_y( T x )
{
	uint8_t i;
	T result;

	// deal with case where ther is no curve
	if( _num_points == 0 ) {
		return x;
	}

	// when x value is lower than the first point's x value, return minimum y value
	if( x <= _x[0] ) {
		return _y[0];
	}

    // when x value is higher than the last point's x value, return maximum y value
	if( x >= _x[_num_points-1] ) {
		return _y[_num_points-1];
	}

	// deal with the normal case
	for( i=0; i<_num_points-1; i++ ) {
		if( x >= _x[i] && x <= _x[i+1] ) {
			result = _y[i] + (x - _x[i]) * _slope[i];
			return result;
		}
	}

	// we should never get here
	return x;
}

// displays the contents of the curve (for debugging)
template <class T, uint8_t SIZE>
void AP_Curve<T,SIZE>::dump_curve()
{
	Serial.println_P(PSTR("Curve:"));
	for( uint8_t i = 0; i<_num_points; i++ ){
		Serial.print_P(PSTR("x:"));
		Serial.print(_x[i]);
		Serial.print_P(PSTR("\ty:"));
		Serial.print(_y[i]);
		Serial.print_P(PSTR("\tslope:"));
		Serial.print(_slope[i],4);
		Serial.println();
	}
}

#endif  // AP_CURVE