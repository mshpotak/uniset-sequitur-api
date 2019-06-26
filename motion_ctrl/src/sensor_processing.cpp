#include "ros/ros.h"
#include "sequitur_pose/SequiturData.h"
#include "motion_ctrl/DriveInfo.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32MultiArray.h"
#include "motion_ctrl/DeadReckoningData.h"
#include "motion_ctrl/SensorProcessingData.h"
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <vector>

typedef geometry_msgs::Point point;
typedef geometry_msgs::Vector3 vector;
typedef std_msgs::Float32MultiArray fl32array;
typedef const sequitur_pose::SequiturData::ConstPtr& seqpointer;
typedef motion_ctrl::DeadReckoningData drdata;
typedef motion_ctrl::SensorProcessingData sdata;

class Stats{
    private:
        double val_sum;
        int data_range;
    public:
        Stats(){
            mean = 0;
            std = 0;
            mean_diff = 0;
            std_diff = 0;
            val_sum = 0;
            data_range = 40;
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
                for ( int i = 0;  i < val.size(); i++ ) {
                    a = val.at(i) - mean;
                    sum = sum + a*a;
                }
                std = sqrt( sum / ( val.size() - 1 ) );
            }
        }
};

class OffsetAccel{
    private:
        float gforce;
        vector a_offset;
        bool init;
    public:
        OffsetAccel(){
            gforce = -9.81;
            init = false;
        }
        void initialize( vector* a ){
            a_offset.x = a->x;
            a_offset.y = a->y;
            a_offset.z = gforce - a->z;
            printPoint( "Accelerometer offset:", a_offset );
        }
        void fix( vector* a ){
            if( init == false ) initialize( a );
            a->x = a->x - a_offset.x;
            a->y = a->y - a_offset.y;
            a->z = a->z - a_offset.z;
        }
};

class OffsetGyro{
    private:
        vector w_offset;
        bool init;
    public:
        OffsetGyro(){
            init = false;
        };
        void initialize( vector* w ){
            w_offset = *w;
            printPoint( "Gyroscope offset:", w_offset );
        }
        void fix( vector* w ){
            if( init == false ) initialize( w );
            w->x = w->x - w_offset.x;
            w->y = w->y - w_offset.y;
            w->z = w->z - w_offset.z;
        }
}

void printPoint( std::string pname, auto point ){
    std::cout << pname << ": "  << point.x << " " << point.y << " " << point.z << "\n";
}

class DeadReckoning{
    private:
        vector a_old;
        vector w_old;
        double t_old;

        vector v;
        double X, Y, Z;
        double roll, pitch, yaw;
        double time;

        double t0;
        bool init;

        double calcVelocity( double v0, double a1, double a2, double t1, double t2 ){
            return v0 + (a2-a1)*(t2-t1);
        }

        double calcDisplacement( double d0, double a1, double a2, double t1, double t2 ){
            return d0 + (a2-a1)*(t2-t1)*(t2-t1)/2;
        }

        double calcAngle( double ang0, double w1, double w2, double t1, double t2 ){
            double ang = ang0 + (w2-w1)*(t2-t1);

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
            v.x = calcVelocity( v.x, a_old.x, a.x, t_old, t );
            v.y = calcVelocity( v.y, a_old.y, a.y, t_old, t );
            v.z = calcVelocity( v.z, a_old.z, a.z, t_old, t );
            X = calcDisplacement( X, a_old.x, a.x, t_old, t );
            Y = calcDisplacement( Y, a_old.y, a.y, t_old, t );
            Z = calcDisplacement( Z, a_old.z, a.z, t_old, t );
            roll  = calcAngle( roll,  w_old.x, w.x, t_old, t );
            pitch = calcAngle( pitch, w_old.y, w.y, t_old, t );
            yaw   = calcAngle( yaw,   w_old.z, w.z, t_old, t );
            a_old = a;
            w_old = w;
            t_old = t;
        }

        vector getVelocity(){
            return v;
        }

        point getDisplacement(){
            point d;
            d.x = X;
            d.y = Y;
            d.z = Z;
            return d;
        }

        vector getAngles(){
            vector ang;
            ang.x = roll;
            ang.y = pitch;
            ang.z = yaw;
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
        Stats ax, ay, az;
        Stats gx, gy, gz;
        Stats mx, my, mz;
        DeadReckoning dr;
        OffsetAccel offsetAccel;
        OffsetGyro offsetGyro;
        vector a, w;
        double t;
    public:
        SensorProccesing(){
            sensor_processing_pub = hand.advertise<sdata>( "sensor_processing", 100 );
            sensor_stats_pub = hand.advertise<fl32array>( "sensor_stats", 100 );
            dr_pub = hand.advertise<drdata>( "dead_reckoning_data", 100 );


            std::cout << "Sensor Processing initialized...\n";
        }

        void update( seqpointer msg ){
            updateStats( msg );
            updateDeadReckoning( msg );
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

            fl32array stats;
            stats.data.clear();

            stats.data.push_back( ax.mean );
            stats.data.push_back( ax.std );
            stats.data.push_back( ay.mean );
            stats.data.push_back( ay.std );
            stats.data.push_back( az.mean );
            stats.data.push_back( az.std );

            stats.data.push_back( gx.mean );
            stats.data.push_back( gx.std );
            stats.data.push_back( gy.mean );
            stats.data.push_back( gy.std );
            stats.data.push_back( gz.mean );
            stats.data.push_back( gz.std );

            stats.data.push_back( mx.mean );
            stats.data.push_back( mx.std );
            stats.data.push_back( my.mean );
            stats.data.push_back( my.std );
            stats.data.push_back( mz.mean );
            stats.data.push_back( mz.std );

            sensor_stats_pub.publish( stats );
        }

        void updateDeadReckoning( seqpointer msg ){
            a = msg->accel.linear;
            w = msg->accel.angular;
            t = msg->header.stamp.toNSec()*pow(10,-9);

            offsetAccel.fix( &a );
            offsetGyro.fix( &w );
            //sensors.filter( a, w, m, t );
            dr.update( a, w, t );

            drdata dr_data;
            ros::Time stamp(dr.getTime());

            dr_data.velocity = dr.getVelocity();
            dr_data.position = dr.getDisplacement();
            dr_data.angle = dr.getAngles();
            dr_data.header.stamp = stamp;

            dr_pub.publish( dr_data );
            updateSensorProcessing( msg, dr_data );
        }

        void updateSensorProcessing( seqpointer msg1, drdata msg2 ){
            sdata sdata;
            sdata.sequitur.header = msg1->header;
            sdata.sequitur.pose = msg1->pose;
            sdata.sequitur.accel = msg1->accel;
            sdata.sequitur.magnetic_field = msg1->magnetic_field;
            sdata.deadreck = msg2;
            sensor_processing_pub.publish( sdata );
        }
};

class SubAndPub{
    private:
        ros::NodeHandle hand;
        ros::Subscriber sequitur_sub;
        SensorProccesing obj;

    public:
        SubAndPub(){
            sequitur_sub = hand.subscribe( "sequitur_data", 20, &SubAndPub::dataCallback, this );
            std::cout << "SubAndPub initialized...\n";
        }
        void dataCallback( const sequitur_pose::SequiturData::ConstPtr& msg ){
            obj.update( msg );
        }
};

int main( int argc, char *argv[]){
    ros::init( argc, argv, "sensor_processing" );
    std::cout << "Node started...\n";
    SubAndPub obj;
    ros::spin();
    return 0;
}
