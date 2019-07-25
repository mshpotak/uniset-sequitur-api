#include "position-proc.hpp"


// PositionFilter class

// Constructor

PositionFilter::PositionFilter(){
    alpha = 0.5;
}

// Private functions

void PositionFilter::detAlpha( float velocity, float min_alpha = 0.01 ){
    alpha = (velocity == 0)*min_alpha + velocity;
};

void PositionFilter::runMovingAvgFilter( DoubleArrayVector new_position ){
    position = alpha*new_position + (1-alpha)*position;
}

// Public functions

void PositionFilter::update( DoubleArrayVector* new_position, double velocity ){
    detAlpha( velocity );
    runMovingAvgFilter( *new_position );
    *new_position = position;
};

DoubleArrayVector PositionFilter::getPosition(){
    return position;
};

// PositionProc class

// Constructor

PositionProc::PositionProc(){
    velocity_old = 0;
    velocity_new = 0;
    t_old = 0;
    alpha = 0.1;
};

void PositionProc::init( double position, double t ){
    t_old = t;
    position_old = position;
};

void PositionProc::update( double position, double t ){
    dt = t - t_old;
    double dx = position - position_old;
    velocity_new = alpha*dx/dt + (1-alpha)*velocity_old;
    t_old = t;
    position_old = position;
};
double PositionProc::getVelocity(){
    return velocity_new;
};
