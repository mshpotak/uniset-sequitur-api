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

void setZero( vector* x ){
    x->x = 0;
    x->y = 0;
    x->z = 0;
}
void setZero( point* x ){
    x->x = 0;
    x->y = 0;
    x->z = 0;
}
void setVector( vector* v, double x, double y, double z ){
    v->x = x;
    v->y = y;
    v->z = z;
}
double vectorMagnitude( double x, double y = 0, double z = 0 ){
    return sqrt( x*x + y*y + z*z );
}
void loadingBar( double fraction ){
    int max_bars = 10;
    int bars = fraction*max_bars;
    std::cout << "Loading [";
    for( int i = 0; i < max_bars; i++ ) {
        if( i < bars ) std::cout << "-";
        else std::cout << " ";
    }
    std::cout << "] " << int(fraction*100) << "%\r";
    std::cout.flush();
}
void printVector( vector v ){
    std::cout << " X: " << v.x << " Y: " << v.y << " Z: " << v.z << "\n";
}

class MagCalibration{
private:
public:
    MagCalibration(){
        x_offset = 0.0442;
        y_offset = 0.0606;
        z_offset = -0.3650;
        sigma = 0.8402;
        theta = 0.7379;
        printf("Magnetometer filtering initialized...\n");
    }
    vector mag_heading;
    double x_offset;
    double y_offset;
    double z_offset;
    double sigma;
    double theta;
    double rot_mat[2][2]  = { {  cos(theta), sin(theta) },
                              { -sin(theta), cos(theta) } };

    double nrot_mat[2][2] = { {  cos(-theta), sin(-theta) },
                              { -sin(-theta), cos(-theta) } };

    void runMagCalibration( seqpointer msg ){
        //remove offset
        double x_hid = msg->magnetic_field.x - x_offset;
        double y_hid = msg->magnetic_field.y - y_offset;
        double z_hid = msg->magnetic_field.z - z_offset;

        //convert to circle
        double x = x_hid;
        double y = y_hid;
        double z = z_hid;

        mag_heading.x = rot_mat[0][0]*x + rot_mat[0][1]*y;
        mag_heading.y = rot_mat[1][0]*x + rot_mat[1][1]*y;
        mag_heading.x = mag_heading.x * sigma;
        mag_heading.x = nrot_mat[0][0]*x + nrot_mat[0][1]*y;
        mag_heading.y = nrot_mat[1][0]*x + nrot_mat[1][1]*y;
        mag_heading.z = z_hid;
    }

    vector getMag(){
        return mag_heading;
    }
};

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
                for ( int i = 0;  i < val.size(); i++ ) {
                    a = val[i] - mean;
                    sum = sum + a*a;
                }
                std = sqrt( sum / ( val.size() - 1 ) );
            }
        }
};

class Threshold{
    private:
        double th;
        double sigma;
    public:
        Threshold(){
            th = 0;
        };
        void setThreshold( double new_th, double new_sigma){
            th = new_th;
            sigma = new_sigma;
        }
        double getTh(){
            return th;
        }
        double runThreshold( double value ){
            if( abs(value) < sigma*th ){
                value = 0;
            }
            //std::cout << "\n" << value << "\t" << th;
            return value;
        }
        double runDiffThreshold( double new_value, double old_value){
            if( abs(new_value - old_value) < (sigma*th) ){
                return old_value;
            }
            return new_value;
        }
};

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

class LowPassFilter{
private:
    double value_old;
    double alpha;
public:
    LowPassFilter(){}

    void initialize( double user_alpha = 0.5, double first_value = 0 ){
        alpha = user_alpha;
        value_old = first_value;
    }

    void update( double* value ){
        *value = alpha*(*value) + (1-alpha)*value_old;
        value_old = *value;
    }
};

class Filtering{
    private:
        Threshold ax, ay, az;
        Threshold gx, gy, gz;
        Threshold mx, my, mz;
        LowPassFilter fmx, fmy, fmz;
        OffsetGyro goff;
    public:
        Filtering(){}
        void setThresholds( double ax_std, double ay_std, double az_std,
                            double gx_std, double gy_std, double gz_std,
                            double mx_std, double my_std, double mz_std ){
            float sigma = 4;
            ax.setThreshold( ax_std, sigma );
            ay.setThreshold( ay_std, sigma );
            az.setThreshold( az_std, sigma );

            gx.setThreshold( gx_std, sigma );
            gy.setThreshold( gy_std, sigma );
            gz.setThreshold( gz_std, sigma );

            mx.setThreshold( mx_std, sigma );
            my.setThreshold( my_std, sigma );
            mz.setThreshold( mz_std, sigma );

            std::cout << " Accl.std:\nx: " << ax.getTh() << "\ny: " << ay.getTh() << "\nz: " << az.getTh() << "\n";
            std::cout << " Gyro.std:\nx: " << gx.getTh() << "\ny: " << gy.getTh() << "\nz: " << gz.getTh() << "\n";
            std::cout << " Magn.std:\nx: " << mx.getTh() << "\ny: " << my.getTh() << "\nz: " << mz.getTh() << "\n";

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
        void runDiffThresholds( vector* a, vector* w, vector* m, vector a_old, vector w_old, vector m_old ){
            a->x = ax.runDiffThreshold( a->x, a_old.x );
            a->y = ay.runDiffThreshold( a->y, a_old.y );
            a->z = az.runDiffThreshold( a->z, a_old.z );

            w->x = gx.runDiffThreshold( w->x, w_old.x );
            w->y = gy.runDiffThreshold( w->y, w_old.y );
            w->z = gz.runDiffThreshold( w->z, w_old.z );

            m->x = mx.runDiffThreshold( m->x, m_old.x );
            m->y = my.runDiffThreshold( m->y, m_old.y );
            m->z = mz.runDiffThreshold( m->z, m_old.z );
        }
        void setOffsets( double gx_mean, double gy_mean, double gz_mean ){
            goff.setOffset( gx_mean, gy_mean, gz_mean );
        }
        void runOffsets( vector* w ){
            goff.runOffset( w );
        }
        void setDriftCorrection( std::vector<double> wx, std::vector<double> wy, std::vector<double> wz, std::vector<double> t ){
            goff.setDriftCorrection( wx, wy, wz, t );
        }
        void runDriftCorrection( vector* w, double t ){
            goff.runDriftCorrection( w, t );
        }
        void setFilters( double alpha, vector m ){
            fmx.initialize( alpha, m.x );
            fmy.initialize( alpha, m.y );
            fmz.initialize( alpha, m.z );
        }
        void runFilters( vector* m ){
            fmx.update( &(m->x) );
            fmy.update( &(m->y) );
            fmz.update( &(m->z) );
        }
};

class GravityVector{
    private:
        double G;
        vector a_grav, a_lin;
        void calcGravityVector( int z_sign, double roll, double pitch ){
            a_grav.x = G*cos(roll);
            a_grav.y = G*cos(pitch);
            float magnitude = G*G - a_grav.x*a_grav.x - a_grav.y*a_grav.y;
            if( magnitude < 0 )
                a_grav.z = 0;
            else
                a_grav.z = z_sign*sqrt(magnitude);
        }
        void calcLinearAccelVector( vector a ){
            a_lin.x = a.x - a_grav.x;
            a_lin.y = a.y - a_grav.y;
            a_lin.z = a.z - a_grav.z;
        }
    public:
        GravityVector(){
            G = 9.8;
        }
        void setGravityVector( double ax, double ay, double az ){
            //G = copysign( vectorMagnitude( ax, ay, az ), az );
            G = vectorMagnitude( ax, ay, az );
        }
        void updateGravityVector( vector a, double roll, double pitch ){
            calcGravityVector( copysign( 1, a.z ), roll, pitch );
            calcLinearAccelVector( a );
        }
        double getGravity(){
            return G;
        }
        vector getGravityVector(){
            return a_grav;
        }
        vector getLinearAccelVector(){
            return a_lin;
        }
};

class DeadReckoning{
    private:
        int upd_number;
        vector a_old;
        vector w_old;
        vector ang_m_old;
        double t_old;
        GravityVector gvector;

        point d;
        vector v, ang;
        vector ang_m;
        vector ang_m_rel;
        vector ang_m_zero;
        vector w_m;
        vector a_lin, a_grav;
        double roll_g, pitch_g, yaw_g;
        double roll_a, pitch_a;
        double time;
        double alpha;

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

        //void correctTilt( vector m, double roll, double pitch ){}
        double calcAngleMag( double y, double x ){
            double z = 0;
            if( abs( x ) < 0.001 ){
                if( y < 0 ){
                    z = 0.5*M_PI;
                } else if ( y > 0 ){
                    z = 1.5*M_PI;
                }
            } else if( x < 0 ){
                    z = M_PI - atan( y/x );
            } else if( x > 0 ){
                if( y < 0 ){
                    z = -atan( y/x );
                } else if ( y > 0 ){
                    z = 2*M_PI - atan( y/x );
                }
            }
            return z;
        }

        vector calcAnglesMag( vector m ){
            vector ang;

            ang.x = calcAngleMag( m.z, m.x ); //roll
            ang.y = calcAngleMag( m.z, m.y ); //pitch
            ang.z = calcAngleMag( m.y, m.x ); //yaw

            //std::cout << "YAW: " << calcAngleMag( m.y, m.x ) << "\n";

            return ang;
        }

        vector calcRelAnglesMag( vector ang_m, vector ang_m0 ){
            vector ang;
            ang.x = AngleDiff( ang_m.x, ang_m0.x);
            ang.y = AngleDiff( ang_m.y, ang_m0.y);
            ang.z = AngleDiff( ang_m.z, ang_m0.z);
            std::cout << "ANG: " << ang_m.z << " ZANG: " << ang_m0.z << "\n";
            return ang;
        }

        vector calcAngleRateMag( vector ang_m_old, vector ang_m, double dt ){
            vector ang_rate;
            ang_rate.x = AngleDiff(ang_m.x, ang_m_old.x)/dt;
            ang_rate.y = AngleDiff(ang_m.y, ang_m_old.y)/dt;
            ang_rate.z = AngleDiff(ang_m.z, ang_m_old.z)/dt;
            return ang_rate;
        }

        double complFilter(  double dt, double g_angle, double a_angle ){
            return alpha*g_angle + (1 - alpha)*a_angle;
        }

        void normilizeVector( vector* a ){
            double sum = sqrt( a->x*a->x + a->y*a->y + a->z*a->z );
            a->x = a->x / sum;
            a->y = a->y / sum;
            a->z = a->z / sum;
        }

        void resizeNorm( vector* norm, double magnitude ){
            norm->x = norm->x * magnitude;
            norm->y = norm->y * magnitude;
            norm->z = norm->z * magnitude;
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

        vector onePiAngles( vector ang ){
            vector new_ang;
            new_ang.x = onePiAngle( ang.x );
            new_ang.y = onePiAngle( ang.y );
            new_ang.z = onePiAngle( ang.z );
            return new_ang;
        }

        double AngleDiff( double angle1, double angle2 ){
            double angle = angle2 - angle1;
            if( angle > M_PI ){
                angle  = angle - 2*M_PI;
            }
            else if( angle < -M_PI ){
                angle = angle + 2*M_PI;
            }
            return -angle;
            //return -((angle1 > angle2) * (2*M_PI - angle2 - angle1) + (angle2 > angle1) * (angle2 - angle1));
        }

        void initialize( vector a, vector w, vector m, double t ){
            gvector.setGravityVector( a.x, a.y, a.z );
            roll_a  = calcAngleAccel( a.x, a.y, a.z );
            pitch_a = calcAngleAccel( a.y, a.x, a.z );
            gvector.updateGravityVector( a, roll_a, pitch_a );
            a_lin  = gvector.getLinearAccelVector();
            ang_m_zero = calcAnglesMag( m );

            ang_m_old  = ang_m_zero;
            a_old = a_lin;
            w_old = w;
            t_old = t;
            t0 = t;
            init = true;
        }

    public:
        DeadReckoning(){
            setZero(&v);
            setZero(&d);
            setZero(&ang);
            setZero(&a_old);
            setZero(&w_old);
            alpha = 0.9;
            t_old = 0;
            init = false;
            t0 = 0;
            time = 0;
            upd_number = 0;
        }

        void update( vector a, vector w, vector m, double t ){
            if( init == false ) {
                initialize( a, w, m, t );
                std::cout << "DEAD RECKONING initialization completed.\n";
                std::cout << "PROGRAM PARAMETERS:\n";
                std::cout << "alpha = [" << alpha << "]\n";
                std::cout << "\n";
                return;
            }
            time = t - t0;
            //std::cout << "UPD #" << upd_number++ << " TIME " << time << "\r";
            //correctAccel( &a, gvector.getGravity() );

            roll_g  = onePiAngle( calcAngleGyro( ang.x, w_old.x, w.x, t_old, t ));
            pitch_g = onePiAngle( calcAngleGyro( ang.y, w_old.y, w.y, t_old, t ));
            yaw_g   = onePiAngle( calcAngleGyro( ang.z, w_old.z, w.z, t_old, t ));
            ang_m     = calcAnglesMag( m );
            ang_m_rel = calcRelAnglesMag ( ang_m, ang_m_zero );
            w_m       = calcAngleRateMag( ang_m_old, ang_m, t - t_old );
            // std::cout << "XANG: " << ang_m.x << " YANG: " << ang_m.y << " ZANG: " << ang_m.z << "\n";
            // std::cout << "XREL: " << ang_m_rel.x << " YREL: " << ang_m_rel.y << " ZREL: " << ang_m_rel.z << "\n";

            roll_a  = calcAngleAccel( a.x, a.y, a.z );
            pitch_a = calcAngleAccel( a.y, a.x, a.z );
            // std::cout << "\nROLL\tGyro angle: " << roll_g << "\n\tAccl angle: " << roll_a << "\n";
            // std::cout << "PTCH\tGyro angle: " << pitch_g << "\n\tAccl angle: " << pitch_a << "\n";

            ang.x = complFilter( t - t_old, roll_g, roll_a );
            ang.y = complFilter( t - t_old, pitch_g, pitch_a );
            ang.z = complFilter( t - t_old, yaw_g, ang_m_rel.z );

            gvector.updateGravityVector( a, ang.x, ang.y );
            a_lin  = gvector.getLinearAccelVector();
            a_grav = gvector.getGravityVector();
            // std::cout << "XLIN: " << a_lin.x << " YLIN: " << a_lin.y << " ZLIN: " << a_lin.z << "\n";
            // std::cout << "XGRV: " << a_grav.x << " YGRV: " << a_grav.y << " ZGRV: " << a_grav.z << " MAGN: " << vectorMagnitude( a_grav.x, a_grav.y, a_grav.z ) <<"\n";

            v.x = calcVelocity( v.x, a_old.x, a_lin.x, t_old, t );
            v.y = calcVelocity( v.y, a_old.y, a_lin.y, t_old, t );
            v.z = calcVelocity( v.z, a_old.z, a_lin.z, t_old, t );

            d.x = calcDisplacement( d.x, v.x, a_old.x, a_lin.x, t_old, t );
            d.y = calcDisplacement( d.y, v.y, a_old.y, a_lin.y, t_old, t );
            d.z = calcDisplacement( d.z, v.z, a_old.z, a_lin.z, t_old, t );

            ang_m_old = ang_m;
            a_old = a_lin;
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

        vector getMagAngles(){
            return ang_m_rel;
        }

};

class SensorProccesing{
    private:
        ros::NodeHandle hand;
        ros::Publisher sensor_stats_pub;
        ros::Publisher dr_pub;
        ros::Publisher sensor_processing_pub;
        ros::Publisher thraccel_pub;
        ros::Publisher magang_pub;
        ros::Publisher command_pub;
        Stats ax, ay, az;
        Stats gx, gy, gz;
        Stats mx, my, mz;
        MagCalibration magcal;
        DeadReckoning dr;
        Filtering filt;
        //OffsetAccel offsetAccel;
        OffsetGyro offsetGyro;
        vector a, w, m;
        vector a_old, w_old, m_old;
        std::vector<double> time;
        double t;
        int statdataN;
    public:
        SensorProccesing( int N = 0 ): ax(N), ay(N), az(N), gx(N), gy(N), gz(N), mx(N), my(N), mz(N) {
            statdataN = N;
            sensor_processing_pub = hand.advertise<sdata>( "sensor_processing_data", 100 );
            thraccel_pub = hand.advertise<vector>( "thraccel", 100 );
            magang_pub = hand.advertise<vector>( "magang", 100 );
            command_pub = hand.advertise<fl32array>( "robot1_commander", 100 );
            std::cout << "Sensor Processing initialized...\n";
        }

        void update( seqpointer msg ){
            updateDeadReckoning( msg );
        }

        bool updateInit( seqpointer msg ){
            loadingBar( (ax.val.size()+1)/double(statdataN) );
            updateStats( msg );
            time.push_back(msg->header.stamp.toNSec()*pow(10,-9));
            if( ax.val.size() >= statdataN ){
                std::cout << "\n";
                filt.setDriftCorrection( gx.val, gy.val, gz.val, time );
                filt.setThresholds( ax.std, ay.std, az.std,
                                    gx.std, gy.std, gz.std,
                                    mx.std, my.std, mz.std );
                magcal.runMagCalibration( msg );
                m_old = magcal.getMag();
                filt.setFilters( 0.2, m_old);
                //filt.setOffsets( gx.mean, gy.mean, gz.mean );
                a_old = msg->accel.linear;
                w_old = msg->accel.angular;

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

            magcal.runMagCalibration( msg );
            m = magcal.getMag();

            mx.update( &m.x );
            my.update( &m.y );
            mz.update( &m.z );

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
            // std::cout << "BEFORE:\n";
            // printVector(a);
            // printVector(w);
            // printVector(m);
            magcal.runMagCalibration( msg );
            m = magcal.getMag();
            filt.runDiffThresholds( &a, &w, &m, a_old, w_old, m_old );
            filt.runThresholds( &a, &w, &m );
            filt.runFilters( &m );
            thraccel_pub.publish ( a );
            // std::cout << "AFTER:\n";
            // printVector(a);
            // printVector(w);
            // printVector(m);
            //filt.runOffsets( &w );
            //offsetAccel.fix( &a );
            dr.update( a, w, m, t );
            magang_pub.publish( dr.getMagAngles() );
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
            a_old = a;
            w_old = w;
            m_old = m;
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
