#include <math.h>

#ifndef LIB_VECTOR_HPP
#define LIB_VECTOR_HPP

struct DoubleArrayVector{
    DoubleArrayVector();
    double x,y,z;
    double magnitude(){
        return sqrt( x*x + y*y + z*z );
    }
};

// double vectorMagnitude( double x, double y = 0, double z = 0 ){
//     return sqrt( x*x + y*y + z*z);
// }

DoubleArrayVector operator*( int lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator*( const DoubleArrayVector& lhs, int rhs );
DoubleArrayVector operator*( float lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator*( const DoubleArrayVector& lhs, float rhs );
DoubleArrayVector operator*( double lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator*( const DoubleArrayVector& lhs, double rhs );
DoubleArrayVector operator*( const DoubleArrayVector& lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator+( int lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator+( const DoubleArrayVector& lhs, int rhs );
DoubleArrayVector operator+( float lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator+( const DoubleArrayVector& lhs, float rhs );
DoubleArrayVector operator+( double lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator+( const DoubleArrayVector& lhs, double rhs );
DoubleArrayVector operator+( const DoubleArrayVector& lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator-( int lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator-( const DoubleArrayVector& lhs, int rhs );
DoubleArrayVector operator-( float lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator-( const DoubleArrayVector& lhs, float rhs );
DoubleArrayVector operator-( double lhs, const DoubleArrayVector& rhs );
DoubleArrayVector operator-( const DoubleArrayVector& lhs, double rhs );
DoubleArrayVector operator-( const DoubleArrayVector& lhs, const DoubleArrayVector& rhs );

#endif
