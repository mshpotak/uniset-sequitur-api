#include "lib-vector.hpp"

DoubleArrayVector::DoubleArrayVector(){
    x = 0;
    y = 0;
    z = 0;
}

DoubleArrayVector operator*( int lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs*rhs.x;
    new_vector.y = lhs*rhs.y;
    new_vector.z = lhs*rhs.z;
    return new_vector;
};
DoubleArrayVector operator*( const DoubleArrayVector& lhs, int rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = rhs*lhs.x;
    new_vector.y = rhs*lhs.y;
    new_vector.z = rhs*lhs.z;
    return new_vector;
};
DoubleArrayVector operator*( float lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs*rhs.x;
    new_vector.y = lhs*rhs.y;
    new_vector.z = lhs*rhs.z;
    return new_vector;
};
DoubleArrayVector operator*( const DoubleArrayVector& lhs, float rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = rhs*lhs.x;
    new_vector.y = rhs*lhs.y;
    new_vector.z = rhs*lhs.z;
    return new_vector;
};
DoubleArrayVector operator*( double lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs*rhs.x;
    new_vector.y = lhs*rhs.y;
    new_vector.z = lhs*rhs.z;
    return new_vector;
};
DoubleArrayVector operator*( const DoubleArrayVector& lhs, double rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = rhs*lhs.x;
    new_vector.y = rhs*lhs.y;
    new_vector.z = rhs*lhs.z;
    return new_vector;
};
DoubleArrayVector operator*( const DoubleArrayVector& lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = rhs.x*lhs.x;
    new_vector.y = rhs.y*lhs.y;
    new_vector.z = rhs.z*lhs.z;
    return new_vector;
};

DoubleArrayVector operator+( int lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs+rhs.x;
    new_vector.y = lhs+rhs.y;
    new_vector.z = lhs+rhs.z;
    return new_vector;
};
DoubleArrayVector operator+( const DoubleArrayVector& lhs, int rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = rhs+lhs.x;
    new_vector.y = rhs+lhs.y;
    new_vector.z = rhs+lhs.z;
    return new_vector;
};
DoubleArrayVector operator+( float lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs+rhs.x;
    new_vector.y = lhs+rhs.y;
    new_vector.z = lhs+rhs.z;
    return new_vector;
};
DoubleArrayVector operator+( const DoubleArrayVector& lhs, float rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = rhs+lhs.x;
    new_vector.y = rhs+lhs.y;
    new_vector.z = rhs+lhs.z;
    return new_vector;
};
DoubleArrayVector operator+( double lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs+rhs.x;
    new_vector.y = lhs+rhs.y;
    new_vector.z = lhs+rhs.z;
    return new_vector;
};
DoubleArrayVector operator+( const DoubleArrayVector& lhs, double rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = rhs+lhs.x;
    new_vector.y = rhs+lhs.y;
    new_vector.z = rhs+lhs.z;
    return new_vector;
};
DoubleArrayVector operator+( const DoubleArrayVector& lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = rhs.x+lhs.x;
    new_vector.y = rhs.y+lhs.y;
    new_vector.z = rhs.z+lhs.z;
    return new_vector;
};

DoubleArrayVector operator-( int lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs-rhs.x;
    new_vector.y = lhs-rhs.y;
    new_vector.z = lhs-rhs.z;
    return new_vector;
};
DoubleArrayVector operator-( const DoubleArrayVector& lhs, int rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs.x-rhs;
    new_vector.y = lhs.y-rhs;
    new_vector.z = lhs.z-rhs;
    return new_vector;
};
DoubleArrayVector operator-( float lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs-rhs.x;
    new_vector.y = lhs-rhs.y;
    new_vector.z = lhs-rhs.z;
    return new_vector;
};
DoubleArrayVector operator-( const DoubleArrayVector& lhs, float rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs.x-rhs;
    new_vector.y = lhs.y-rhs;
    new_vector.z = lhs.z-rhs;
    return new_vector;
};
DoubleArrayVector operator-( double lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs-rhs.x;
    new_vector.y = lhs-rhs.y;
    new_vector.z = lhs-rhs.z;
    return new_vector;
};
DoubleArrayVector operator-( const DoubleArrayVector& lhs, double rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs.x-rhs;
    new_vector.y = lhs.y-rhs;
    new_vector.z = lhs.z-rhs;
    return new_vector;
};
DoubleArrayVector operator-( const DoubleArrayVector& lhs, const DoubleArrayVector& rhs ){
    DoubleArrayVector new_vector;
    new_vector.x = lhs.x-rhs.x;
    new_vector.y = lhs.y-rhs.y;
    new_vector.z = lhs.z-rhs.z;
    return new_vector;
};
