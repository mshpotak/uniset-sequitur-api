#include <math.h>
#include "lib-vector.hpp"

#ifndef IMU_PROC_HPP
#define IMU_PROC_HPP

class IMUFilter{
private:
    double roll, pitch, yaw;
    double detAngleWithGyro( double angle, double w1, double w2, double t1, double t2 );
    double detAngleWithAccel( double hor_axis, double ort_axis, double g_axis );
    void runComplFilter( double* filt_angle, float alpha,  double gyro_angle, double accel_angle );
public:
    IMUFilter();
    void update();
    double getRoll();
    double getPitch();
    double getYaw();
};

#endif
