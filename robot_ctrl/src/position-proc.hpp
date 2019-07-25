#include <math.h>
#include <iostream>
#include "imu-proc.hpp"
#include "lib-vector.hpp"

#ifndef POSITION_PROC_HPP
#define POSITION_PROC_HPP

class PositionFilter{
private:
    DoubleArrayVector position;
    double alpha; // position averaging coeficient

    void detAlpha( float velocity, float min_alpha );

    void runMovingAvgFilter( DoubleArrayVector new_position );


public:
    PositionFilter();

    void update( DoubleArrayVector* new_position, double velocity );

    DoubleArrayVector getPosition();
};

class PositionProc{
private:
    double position_old;
    double velocity_old;
    double velocity_new;
    double t_old;
    double dt;
    double alpha;

public:
    PositionProc();
    void init( double position, double t );
    void update( double position, double t );
    double getVelocity();
};

#endif
