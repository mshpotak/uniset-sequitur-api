#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/AccelStamped.h"
#include "tf/transform_datatypes.h"

#include <stdlib.h>
#include <math.h>

struct Point2D{
    Point2D(){}
    Point2D( double i, double j): x(i), y(j){}
    double x;
    double y;
    // inline Point2D operator-(Point2D lhs, const Point2D& rhs){
    //     lhs.x -= rhs.x;
    //     lhs.y -= rhs.y;
    //     return lhs;
    // }
};

struct Square2D{
    Point2D top_l;
    Point2D top_r;
    Point2D bot_l;
    Point2D bot_r;
    double len_vrt(){
        return abs(top_l.y - bot_l.y);
    }
    double len_hor(){
        return abs(bot_l.x - bot_r.x);
    }
};

class Grid{
    public:
        Grid(){
            set_box();
            printf("Grid initialized...\n");
        };
        Grid( int qds_vrt, int qds_hor ){
            set_box();
            make_grid( qds_vrt, qds_hor );
            printf("Grid initialized...\n");
        }
        Square2D fin;
        Square2D brd;

        void fin_grid( double x, double y ){
            fin.bot_r.x = find_range( x, brd.bot_l.x, brd.bot_r.x, step_hor );
            fin.bot_l.x = fin.bot_r.x - step_hor;

            fin.top_l.y = find_range( y, brd.bot_l.y, brd.top_l.y, step_vrt );
            fin.bot_l.y = fin.top_l.y - step_vrt;

            fin.top_l.x = fin.bot_l.x;
            fin.bot_r.y = fin.bot_l.y;
            fin.top_r.y = fin.top_l.y;
            fin.top_r.x = fin.bot_r.x;
        }

        int if_in( const double *x, const double *y, const Square2D *grid ){
            if( *x > grid->bot_l.x && *y > grid->bot_l.y &&
                *x < grid->top_r.x && *y < grid->top_r.y ){
                return 1;
            }else return 0;
        }

        void make_grid( int quadrants_vrt, int quadrants_hor ){
            step_vrt = brd.len_vrt()/quadrants_vrt;
            step_hor = brd.len_hor()/quadrants_hor;
        }

        void set_box(){
            brd.top_l.y = 1.7;
            brd.top_l.x = -1.2;
            brd.top_r.y = 2;
            brd.top_r.x = 2.9;
            brd.bot_l.y = -1.6;
            brd.bot_l.x = -1.2;
            brd.bot_r.y = -1.6;
            brd.bot_r.x = 2.9;
        }
    private:
        double step_vrt;
        double step_hor;

        double find_range( double x, double start, double limit, double step ){
            double max = start + step;
            if( x <= start ){
                return max;
            }
            while( x > max ){
                if( max >= limit ){
                    return limit;
                }
                max += step;
            }
            return max;
        }
};

// class DataFilter{
//     public:
//         DataFilter(){
//             north = 0.19;
//             west = -0.48;
//             east = 0.59;
//             south = -0.17;
//         }
//         double north;
//         double east;
//         double south;
//         double west;
// };
//
// struct Vector{
//     public:
//         Vector(xyz a, xyz b){
//             vect_to.x = a.x;
//             vect_to.y = a.y;
//             vect_fr.x = b.x;
//             vect_fr.y = b.y;
//
//         }
//         Point2D vect_to, vect_fr;
//         double vers;
//     private:
//         angle(){
//             Point2D i( 1, 0 );
//             Point2D vect = vect_to - vect_fr;
//
//         }
// }

class RovMoPlan{
    private:
        int databank;
        int db_vect;
        int db_vers;

        int act_pending;;
        Point2D dest;
        Point2D av_pos;
        Point2D av_mag;
        double rel_ang;
        double dest_ang;
        double dist;

        std_msgs::Int8 fwd;
        std_msgs::Int8 stop;
        std_msgs::Int8 left;
        std_msgs::Int8 right;

        void reset(){
                db_vect = 0;
                db_vers = 0;
                act_pending = 0;
                av_pos.x = 0;
                av_pos.y = 0;
                av_mag.x = 0;
                av_mag.y = 0;
        }

    public:
        RoverMoPlan(){
            fwd.data = 1;
            stop.data = 0;
            right.data = 4;
            left.data = 3;
            reset();
        };
        int act( const double *x, const double *y ){
            dest.x = x;
            dest.y = y;
            if( db_vect > 50 && db_vers > 50 ){
                rotate();
                go();
                reset();
                return 0;
            } else
                return ++act_pending;
        }
        void process( geometry_msgs::PoseStamped::ConstPtr& msg ){
            av_pos.x += msg->pose.position.x;
            av_pos.y += msg->pose.position.y;
            db_vect++;
        }
        void process( sensor_msgs::MagneticField::ConstPtr& msg ){
            av_mag.x += msg->magnetic_field.x;
            av_mag.y += msg->magnetic_field.y;
            db_vers++;
        }
        void process(){
            av_pos.x /= db_vect;
            av_pos.y /= db_vect;
            av_mag.x /= db_vers;
            av_mag.y /= db_vers;
            rel_ang = atan2( av_mag.y, av_mag.x );
            dest_ang = atan2( av_pos.y, av_pos.x );
            dist = sqrt( av_pos.x*av_pos.x + av_pos.y*av_pos.y );

            //if( av_ang < 0 ) av_ang += 2*M_PI;
        }
        //void process( geometry_msgs::AccelStamped::ConstPtr& msg ){}
        void rotate(){
            //printf("Yaw is: %f\n", yaw);
            if( av_ang > 0 ){
                if( yaw > teta + 0.15 ) {
                    ctrl_pub.publish( left );
                }else if( yaw < teta - 0.15 ) {
                    ctrl_pub.publish( right );
            }
        };
        void go(){


            //printf("Control set...\n");
            actual_pose = *msg;
            //printf("Finish grid set...\n");

            dist_x = p_fin_pose->pose.position.x - msg->pose.position.x;
            dist_y = p_fin_pose->pose.position.y - msg->pose.position.y;
            if( dist_x == 0 && dist_y == 0 ){
                ctrl_pub.publish( stop );
            }else if ( grid_obj.if_in( &msg->pose.position.x, &msg->pose.position.y, &grid_obj.brd ) == 0) {
                ctrl_pub.publish( stop );
                printf("Out of bounds.\n");
            }else if ( grid_obj.if_in( &msg->pose.position.x, &msg->pose.position.y, &grid_obj.fin ) == 1 ) {
                ctrl_pub.publish( stop );
                printf("Destination reached.\n");
            }else {
                norme = sqrt( dist_x*dist_x + dist_y*dist_y );
                teta = atan2( dist_x, dist_y );
                if( teta < 0 ) teta += 2*M_PI;
                printf("Teta: %f\t Yaw: %f\n", teta, yaw);



                }else if( norme > 0.1 ){
                    ctrl_pub.publish( fwd );
                } else {
                    ctrl_pub.publish( stop );
                }
            }

        };
};

class SubAndPub{
    public:
        SubAndPub(){
            ctrl_pub = hand.advertise<std_msgs::Int8>( "rover_command", 200 );
            pose_sub = hand.subscribe( "sequitur_pose", 200, &SubAndPub::seqCallback, this );
            mag_sub = hand.subscribe("sequitur_mag", 200, &SubAndPub::magCallback, this );
            //accel_sub = hand.subscribe("sequitur_accel", 200, &SubAndPub::accelCallback, this );
            ctrl_sub = hand.subscribe( "rover_destination", 200, &SubAndPub::destCallback, this);
            grid_obj.make_grid( 5, 6);
            printf("SubAndPub initialized...\n");
            p_fin_pose = &actual_pose;
            grid_obj.fin_grid( p_fin_pose->pose.position.x, p_fin_pose->pose.position.y );
        }

        void seqCallback( const geometry_msgs::PoseStamped::ConstPtr& msg ){
            rover.process( msg );
        }

        void magCallback( const sensor_msgs::MagneticField::ConstPtr& msg ){
            rover.process( msg );
        }


        void destCallback( const geometry_msgs::PoseStamped::ConstPtr& msg ){
            fin_pose = *msg;
            p_fin_pose = &fin_pose;
            grid_obj.fin_grid( p_fin_pose->pose.position.x, p_fin_pose->pose.position.y );
            rover.act();
        }


    private:
        ros::NodeHandle hand;
        ros::Publisher ctrl_pub;
        ros::Subscriber pose_sub;
        ros::Subscriber ctrl_sub;
        ros::Subscriber mag_sub;
        RovMoPlan rover;
        //ros::Subscriber accel_sub;
        geometry_msgs::PoseStamped actual_pose;
        geometry_msgs::PoseStamped fin_pose;
        geometry_msgs::PoseStamped *p_fin_pose;
        double dist_x;
        double dist_y;
        double norme;
        double teta;
        double yaw;
        Grid grid_obj;
};

int main( int argc, char *argv[] ){
    ros::init( argc, argv, "rover_ark_ctrl" );
    SubAndPub ark_obj;
    ros::spin();
    return 0;
}
