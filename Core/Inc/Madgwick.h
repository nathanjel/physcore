//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>


struct Quaternion {
	float w, x, y, z;
};

template <class T>
class V3 {
public:
	T x, y, z;
	V3(T _x, T _y, T _z) : x{_x}, y{_y}, z{_z} {};
	V3(const V3& other) : x{other.x}, y{other.y}, z{other.z} {};
	V3& operator=(const V3& other) {
		x = other.x;
		y = other.y;
		z = other.z;
		return *this;
	};
	V3& operator+=(const V3& other) {
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}
	V3& operator*=(const T other) {
		x *= other;
		y *= other;
		z *= other;
		return *this;
	}
	V3& operator*=(const V3& other) {
		// https://en.wikipedia.org/wiki/Cross_product#Coordinate_notation
		T _x = x;
		T _y = y;
		T _z = z;
		x = _y * other.z - _z * other.y;
		y = _z * other.x - _x * other.z;
		z = _x * other.y - _y * other.x;
		return *this;
	}
	friend V3 operator+(V3 lhs, const V3& other) {
		lhs += other;
		return lhs;
	}
	friend V3 operator*(V3 lhs, const T other) {
		lhs *= other;
		return lhs;
	}
	friend V3 operator*(V3 lhs, const V3& other) {
		lhs *= other;
		return lhs;
	}
	V3& rotateBy(const Quaternion& other) {
		// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Vector_rotation
		//		{\displaystyle {\vec {t}}=2{\vec {q}}\times {\vec {v}}}
		//		{\displaystyle {\vec {v}}^{\,\prime }={\vec {v}}+q_{0}{\vec {t}}+{\vec {q}}\times {\vec {t}}}{\displaystyle {\vec {v}}^{\,\prime }={\vec {v}}+q_{0}{\vec {t}}+{\vec {q}}\times {\vec {t}}}
		V3 q(other.x, other.y, other.z);
		V3 t = ((q*(float)2)) * *this;
		*this = *this + (t * other.w) + (q * t);
		return *this;
	}
};

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    //float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    //float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    //float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
    Quaternion getInternalQ() {
    	Quaternion p = {
    		q0, q1, q2, q3
    	};
    	return p;
    }
};
#endif
