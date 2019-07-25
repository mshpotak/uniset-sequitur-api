#include "imu-proc.hpp"

double IMUFilter::detAngleGyro( double angle, double w1, double w2, double t1, double t2 ){
    return angle + ((w2+w1)/2)*(t2-t1);
}

double IMUFilter::detAngleAccel( double hor_axis, double ort_axis, double g_axis ){
    return atan2( sqrt( ort_axis*ort_axis + g_axis*g_axis ), hor_axis );
}

void IMUFilter::runComplFilter( double* filt_angle, float alpha,  double gyro_angle, double accel_angle ){
    *filt_angle = (1 - alpha)*gyro_angle + alpha*accel_angle;
}

void IMUFilter::update(){}

double IMUFilter::getRoll(){
    return roll;
}

double IMUFilter::getPitch(){
    return pitch;
}

double IMUFilter::getYaw(){
    return yaw;
}
