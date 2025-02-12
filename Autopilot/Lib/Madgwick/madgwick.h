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

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static float invSqrt(float x);
    float _beta;
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(float dt, float beta);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void updateGyro(float gx, float gy, float gz);
    float get_q0() { return q0; };
    float get_q1() { return q1; };
    float get_q2() { return q2; };
    float get_q3() { return q3; };
    void set_state(float q0_, float q1_, float q2_, float q3_);

    float getRoll() {
        computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians() {
        computeAngles();
        return roll;
    }
    float getPitchRadians() {
        computeAngles();
        return pitch;
    }
    float getYawRadians() {
        computeAngles();
        return yaw;
    }
};
#endif

