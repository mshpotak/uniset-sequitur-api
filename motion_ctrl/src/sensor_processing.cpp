#include "ros/ros.h"
#include "sequitur_pose/SequiturData.h"
#include "motion_ctrl/DriveInfo.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32MultiArray.h"
#include "motion_ctrl/DeadReckoningData.h"
#include "motion_ctrl/SensorProcessingData.h"
#include "motion_ctrl/IMUStats.h"
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <vector>

typedef geometry_msgs::Point point;
typedef geometry_msgs::Vector3 vector;
typedef std_msgs::Float32MultiArray fl32array;
typedef sequitur_pose::SequiturData seqdata;
typedef const sequitur_pose::SequiturData::ConstPtr& seqpointer;
typedef motion_ctrl::DeadReckoningData drdata;
typedef motion_ctrl::SensorProcessingData sdata;
typedef motion_ctrl::IMUStats imustats;

class Stats{
    private:
        double val_sum;
        int data_range;
    public:
        Stats( int input_data_range = 40){
            mean = 0;
            std = 0;
            mean_diff = 0;
            std_diff = 0;
            val_sum = 0;
            data_range = input_data_range;
        }

        std::vector<double> val;
        double mean;
        double std;
        double mean_diff;
        double std_diff;

        void update( const double* x){
            val.push_back( *x );
            val_sum = val_sum + *x;
            if( val.size() > data_range ){
                val_sum = val_sum - val[val.size() - data_range];
            }

            double mean_old = mean;
            update_mean( x );
            mean_diff = mean - mean_old;

            double std_old = std;
            update_std( x );
            std_diff = std - std_old;
        }

        void update_mean( const double* x){
            mean = val_sum / data_range;
        }

        void update_std( const double* x ){
            if( val.size() > 2 ){
                double sum = 0;
                double a = 0;
                for ( int i = val.size()-data_range;  i < val.size(); i++ ) {
                    a = val.at(i) - mean;
                    sum = sum + a*a;
                }
                std = sqrt( sum / ( val.size() - 1 ) );
            }
        }
};

// class Calibration{
//
// };

class Threshold{
    private:
        double th;
    public:
        Threshold(){
            th = 0;
        };
        void setThreshold( double new_th ){
            th = new_th;
        }
        double getTh(){
            return th;
        }
        double runThreshold( double value ){
            if( abs(value) < 3*th ){
                value = 0;
            }
            //std::cout << "\n" << value << "\t" << th;
            return value;
        }
};

// class OffsetAccel{
//     private:
//         float gforce;
//         vector a_offset;
//         bool init;
//     public:
//         OffsetAccel(){
//             gforce = -9.81;
//             init = false;
//         }
//         void initialize( vector* a ){
//             a_offset.x = a->x;
//             a_offset.y = a->y;
//             a_offset.z = gforce - a->z;
//             printPoint( "Accelerometer offset:", a_offset );
//             init = true;
//         }
//         void fix( vector* a ){
//             if( init == false ) initialize( a );
//             a->x = a->x - a_offset.x;
//             a->y = a->y - a_offset.y;
//             a->z = a->z - a_offset.z;
//         }
//         void printPoint( std::string pname, auto point ){
//             std::cout << pname << ": "  << point.x << " " << point.y << " " << point.z << "\n";
//         }
// };

class OffsetGyro{
    private:
        vector w_offset;
        vector a_w;
        vector b_w;
        bool init;
    public:
        OffsetGyro(){
            w_offset.x = 0;
            w_offset.y = 0;
            w_offset.z = 0;
            init = false;
        };
        void setOffset( vector* w ){
            w_offset = *w;
            printPoint( "Gyroscope offset:", w_offset );
            init = true;
        }
        void setOffset( double x, double y, double z ){
            w_offset.x = x;
            w_offset.y = y;
            w_offset.z = z;
            printPoint( "Gyroscope offset:", w_offset );
            init = true;
        }
        void runOffset( vector* w ){
            if( init == false ) setOffset( w );
            w->x = w->x - w_offset.x;
            w->y = w->y - w_offset.y;
            w->z = w->z - w_offset.z;
        }
        void printPoint( std::string pname, auto point ){
            std::cout << pname << ": "  << point.x << " " << point.y << " " << point.z << "\n";
        }

        void setDriftCorrection( std::vector<double> wx, std::vector<double> wy, std::vector<double> wz, std::vector<double> t ){
            fitLeastSquares( t, wx, &(a_w.x), &(b_w.x) );
            fitLeastSquares( t, wy, &(a_w.y), &(b_w.y) );
            fitLeastSquares( t, wz, &(a_w.z), &(b_w.z) );
            std::cout << "X drift:" << a_w.x << "\t" << b_w.x << "\n";
            std::cout << "Y drift:" << a_w.y << "\t" << b_w.y << "\n";
            std::cout << "Z drift:" << a_w.z << "\t" << b_w.z << "\n";
        }

        void runDriftCorrection( vector* w, double t ){
            w->x = w->x - (a_w.x*t + b_w.x);
            w->y = w->y - (a_w.y*t + b_w.y);
            w->z = w->z - (a_w.z*t + b_w.z);
        }

        void fitLeastSquares( std::vector<double> x, std::vector<double> y, double* a, double* b ){
            int n = x.size();
            double xsum = 0, x2sum = 0, ysum = 0, xysum = 0;
            for ( int i = 0; i < n; i++ ){
                  xsum = xsum + x[i];               //calculate sigma(xi)
                  ysum = ysum + y[i];               //calculate sigma(yi)
                  x2sum = x2sum + pow( x[i], 2 );   //calculate sigma(x^2i)
                  xysum = xysum + x[i]*y[i];        //calculate sigma(xi*yi)
                }
            *a = (n*xysum-xsum*ysum)/(n*x2sum-xsum*xsum);     //calculate slope
            *b = (x2sum*ysum-xsum*xysum)/(x2sum*n-xsum*xsum); //calculate intercept
        }
};

class Filtering{
    private:
        Threshold ax, ay, az;
        Threshold gx, gy, gz;
        Threshold mx, my, mz;
        OffsetGyro goff;
    public:
        Filtering(){}
        void setThresholds( double ax_std, double ay_std, double az_std,
                            double gx_std, double gy_std, double gz_std,
                            double mx_std, double my_std, double mz_std ){

            ax.setThreshold( ax_std );
            ay.setThreshold( ay_std );
            az.setThreshold( az_std );

            gx.setThreshold( gx_std );
            gy.setThreshold( gy_std );
            gz.setThreshold( gz_std );

            mx.setThreshold( mx_std );
            my.setThreshold( my_std );
            mz.setThreshold( mz_std );

            std::cout << " Accl.std:\nx: " << ax.getTh() << "\ny: " << ay.getTh() << "\nz: " << az.getTh() << "\n";
            std::cout << " Gyro.std:\nx: " << gx.getTh() << "\ny: " << gy.getTh() << "\nz: " << gz.getTh() << "\n";
            std::cout << " Magn.std:\nx: " << mx.getTh() << "\ny: " << my.getTh() << "\nz: " << mz.getTh() << "\n";

        }
        void setOffsets( double gx_mean, double gy_mean, double gz_mean ){
            goff.setOffset( gx_mean, gy_mean, gz_mean );
        }
        void runOffsets( vector* w ){
            goff.runOffset( w );
        }
        void runThresholds( vector* a, vector* w, vector* m ){
            a->x = ax.runThreshold( a->x );
            a->y = ay.runThreshold( a->y );
            a->z = az.runThreshold( a->z );

            w->x = gx.runThreshold( w->x );
            w->y = gy.runThreshold( w->y );
            w->z = gz.runThreshold( w->z );

            m->x = mx.runThreshold( m->x );
            m->y = my.runThreshold( m->y );
            m->z = mz.runThreshold( m->z );
        }
        void setDriftCorrection( std::vector<double> wx, std::vector<double> wy, std::vector<double> wz, std::vector<double> t ){
            goff.setDriftCorrection( wx, wy, wz, t );
        }
        void runDriftCorrection( vector* w, double t ){
            goff.runDriftCorrection( w, t );
        }
};

class DeadReckoning{
    private:
        vector a_old;
        vector w_old;
        double t_old;

        vector v, d, ang;
        double roll_g, pitch_g, yaw_g;
        double roll_a, pitch_a;
        double alpha_roll, alpha_pitch;
        double time;

        double t0;
        bool init;

        double calcVelocity( double v0, double a1, double a2, double t1, double t2 ){
            return v0 + ((a2+a1)/2)*(t2-t1);
        }

        double calcDisplacement( double d0, double v0, double a1, double a2, double t1, double t2 ){
            return d0 + (t2-t1)*( v0 + ((a2+a1)/2)*(t2-t1)/2 );
        }

        double calcAngleGyro( double ang0, double w1, double w2, double t1, double t2 ){
            return ang0 + ((w2+w1)/2)*(t2-t1) ;
        }

        double calcAngleAccel( double hor_axis, double ort_axis, double g_axis ){
            return atan2( sqrt( ort_axis*ort_axis + g_axis*g_axis ), hor_axis );
        }

        double complFilter(  double alpha, double g_angle, double a_angle ){
            return (1 - alpha)*g_angle + alpha*a_angle;
        }

        void normilizeVector( vector* a ){
            double sum = sqrt( a->x*a->x + a->y*a->y + a->z*a->z );
            a->x = a->x / sum;
            a->y = a->y / sum;
            a->z = a->z / sum;
        }

        void resizeNorm( vector* norm, double magnitude ){
            a_norm->x = a_norm->x * gforce;
            a_norm->y = a_norm->y * gforce;
            a_norm->z = a_norm->z * gforce;
        }

        void correctAccel( vector *a, double gforce = 9.81 ){
            normilizeVector( a );
            resizeNorm( a, gforce );
        }

        double onePiAngle( double ang ){
            if( abs(ang) > 2*M_PI ){
                int k = ang/(2*M_PI);
                ang = ang - k*2*M_PI;
            }
            if( abs(ang) > M_PI ){
                ang = ang + copysign(2*M_PI,-ang);
            }
            return ang;
        }

        void initialize( vector a, vector w, double t ){
            a_old = a;
            w_old = w;
            t_old = t;
            t0 = t;
            init = true;
        }

    public:
        DeadReckoning(){
            v.x = 0;
            v.y = 0;
            v.z = 0;
            X = 0;
            Y = 0;
            Z = 0;
            roll = 0;
            pitch = 0;
            yaw = 0;
            a_old.x = 0;
            a_old.y = 0;
            a_old.z = 0;
            w_old.x = 0;
            w_old.y = 0;
            w_old.z = 0;
            t_old = 0;
            init = false;
            t0 = 0;
            time = 0;
        }

        void update( vector a, vector w, double t ){
            if( init == false ) initialize( a, w, t );

            time = t - t0;

            correctAccel( a );

            v.x = calcVelocity( v.x, a_old.x, a.x, t_old, t );
            v.y = calcVelocity( v.y, a_old.y, a.y, t_old, t );
            v.z = calcVelocity( v.z, a_old.z, a.z, t_old, t );

            d.x = calcDisplacement( d.x, v.x, a_old.x, a.x, t_old, t );
            d.y = calcDisplacement( d.y, v.y, a_old.y, a.y, t_old, t );
            d.z = calcDisplacement( d.z, v.z, a_old.z, a.z, t_old, t );

            roll_g  = onePiAngle( calcAngleGyro( ang.x, w_old.x, w.x, t_old, t ));
            pitch_g = onePiAngle( calcAngleGyro( ang.y, w_old.y, w.y, t_old, t ));
            yaw_g   = onePiAngle( calcAngleGyro( ang.z, w_old.z, w.z, t_old, t ));

            roll_a  = onePiAngle( calcAngleAccel( a.x, a.y, a.z ));
            pitch_a = onePiAngle( calcAngleAccel( a.y, a.x, a.z ));

            ang.x = complFilter( alpha_roll, roll_g, roll_a );
            ang.y = complFilter( pitch_alpha, pitch_g, pitch_a );
            ang.z = yaw_g;

            a_old = a;
            w_old = w;
            t_old = t;
        }

        vector getVelocity(){
            return v;
        }

        point getDisplacement(){
            return d;
        }

        vector getAngles(){
            return ang;
        }

        double getTime(){
            return time;
        }

};

class SensorProccesing{
    private:
        ros::NodeHandle hand;
        ros::Publisher sensor_stats_pub;
        ros::Publisher dr_pub;
        ros::Publisher sensor_processing_pub;
        ros::Publisher fgyro_pub;
        Stats ax, ay, az;
        Stats gx, gy, gz;
        Stats mx, my, mz;
        DeadReckoning dr;
        Filtering filt;
        //OffsetAccel offsetAccel;
        OffsetGyro offsetGyro;
        vector a, w, m;
        std::vector<double> time;
        double t;
        int statdataN;
    public:
        SensorProccesing( int N = 0 ): ax(N), ay(N), az(N), gx(N), gy(N), gz(N), mx(N), my(N), mz(N) {
            statdataN = N;
            sensor_processing_pub = hand.advertise<sdata>( "sensor_processing", 100 );
            fgyro_pub = hand.advertise<vector>( "fgyro", 100 );
            std::cout << "Sensor Processing initialized...\n";
        }

        void update( seqpointer msg ){
            updateDeadReckoning( msg );
        }

        bool updateInit( seqpointer msg ){
            updateStats( msg );
            time.push_back(msg->header.stamp.toNSec()*pow(10,-9));
            if( ax.val.size() >= statdataN ){
                filt.setDriftCorrection( gx.val, gy.val, gz.val, time );
                filt.setThresholds( ax.std, ay.std, az.std,
                                    gx.std, gy.std, gz.std,
                                    mx.std, my.std, mz.std );
                filt.setOffsets( gx.mean, gy.mean, gz.mean );
                return true;
            }
            return false;
        }

        void updateStats( seqpointer msg ){
            ax.update( &msg->accel.linear.x );
            ay.update( &msg->accel.linear.y );
            az.update( &msg->accel.linear.z );

            gx.update( &msg->accel.angular.x );
            gy.update( &msg->accel.angular.y );
            gz.update( &msg->accel.angular.z );

            mx.update( &msg->magnetic_field.x );
            my.update( &msg->magnetic_field.y );
            mz.update( &msg->magnetic_field.z );

            // imustats stats;
            //
            // stats.a_mean.x = ax.mean;
            // stats.a_mean.y = ay.mean;
            // stats.a_mean.z = az.mean;
            // stats.g_mean.x = gx.mean;
            // stats.g_mean.y = gy.mean;
            // stats.g_mean.z = gz.mean;
            // stats.m_mean.x = mx.mean;
            // stats.m_mean.y = my.mean;
            // stats.m_mean.z = mz.mean;
            //
            // stats.a_std.x = ax.std;
            // stats.a_std.y = ay.std;
            // stats.a_std.z = az.std;
            // stats.g_std.x = gx.std;
            // stats.g_std.y = gy.std;
            // stats.g_std.z = gz.std;
            // stats.m_std.x = mx.std;
            // stats.m_std.y = my.std;
            // stats.m_std.z = mz.std;
        }

        void updateDeadReckoning( seqpointer msg ){
            a = msg->accel.linear;
            w = msg->accel.angular;
            m = msg->magnetic_field;
            t = msg->header.stamp.toNSec()*pow(10,-9);
            filt.runDriftCorrection( &w, t );
            filt.runThresholds( &a, &w, &m );
            //filt.runOffsets( &w );
            //offsetAccel.fix( &a );
            dr.update( a, w, t );

            sdata sdata;

            sdata.header = sdata.header;

            sdata.sequitur.header = msg->header;
            sdata.sequitur.pose = msg->pose;
            sdata.sequitur.accel.linear = a;
            sdata.sequitur.accel.angular = w;
            sdata.sequitur.magnetic_field = m;

            sdata.deadreck.header.stamp = msg->header.stamp;
            sdata.deadreck.velocity = dr.getVelocity();
            sdata.deadreck.position = dr.getDisplacement();
            sdata.deadreck.angle = dr.getAngles();

            sensor_processing_pub.publish( sdata );
        }

};

class SubAndPub{
    private:
        ros::NodeHandle hand;
        ros::Subscriber sequitur_sub;
        SensorProccesing obj;

    public:
        SubAndPub(): obj(1000){
            sequitur_sub = hand.subscribe( "sequitur_data", 20, &SubAndPub::initCallback, this );
            std::cout << "SubAndPub initialized...\n";
        }
        void dataCallback( const sequitur_pose::SequiturData::ConstPtr& msg ){
            obj.update( msg );
        }
        void initCallback( const sequitur_pose::SequiturData::ConstPtr& msg ){
            if( obj.updateInit( msg ) ){
                sequitur_sub = hand.subscribe( "sequitur_data", 20, &SubAndPub::dataCallback, this );
                std::cout << "Calibrations finished.\n";
            }
        }
};

int main( int argc, char *argv[]){
    ros::init( argc, argv, "sensor_processing" );
    std::cout << "Node started...\n";
    SubAndPub obj;
    ros::spin();
    return 0;
}
