#include "ros/ros.h"
#include "sequitur_pose/SequiturData.h"
#include "motion_ctrl/DriveInfo.h"
#include "motion_ctrl/SensorProcessingData.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32MultiArray.h"
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <vector>

typedef geometry_msgs::Point point;

class MotionControl{
private:
    ros::NodeHandle hand;
    ros::Publisher info_pub;
    ros::Publisher command_pub;
    std::vector<geometry_msgs::Point> waypoint;
    std::vector<geometry_msgs::Point> past_positions;
    int N;
    bool sleep;
    float pos_accuracy;
    float direction;
    float distance;
    float fs;
public:
    MotionControl(){
        info_pub = hand.advertise<motion_ctrl::DriveInfo>( "drive_info", 10 );
        command_pub = hand.advertise<std_msgs::Float32MultiArray>( "robot1_commander", 10 );
        N = 0;
        sleep = true;
        pos_accuracy = 0.3;
        direction = 0;
        distance = 0;
        std::cout << "Motion Control initialized...\n";
        fs = 18;
    }

    void updateWay( const geometry_msgs::Point::ConstPtr& new_point ){
        waypoint.push_back( *new_point );
        std::cout << "New waypoint x: [" << waypoint.back().x << "], y: [" << waypoint.back().y << "] recieved.\n";
        sleep = false;
    }

    void updatePose( const motion_ctrl::SensorProcessingData::ConstPtr& msg ){
        if( sleep == false ){
            //std::cout << "Pose update recieved.\n";
            if( !checkGroundBoundaries( msg->sequitur.pose.position ) ){
                destReached();
                return;
            }

            float distance = calcDistance( msg->sequitur.pose.position, waypoint[N] );
            float direction = calcDirection( msg->sequitur.pose.position, waypoint[N] );
            float orientation_imu = msg->deadreck.angle.z;
            float orientation_pos = calcOrientation( msg->sequitur.pose.position );

            if ( (distance - pos_accuracy) < 0 ) {
                destReached();
                sendInfo( distance, direction, orientation_imu );
                return;
            }
            sendCommand( distance, direction, orientation_imu );
            sendInfo( distance, direction, orientation_imu );
        }
    }

    float calcDistance( const geometry_msgs::Point current_pos, const geometry_msgs::Point desired_pos ){
        float x = desired_pos.x - current_pos.x;
        std::cout << "D: " << desired_pos.x << " C: " << current_pos.x << "\n";
        float y = desired_pos.y - current_pos.y;
        std::cout << "D: " << desired_pos.y << " C: " << current_pos.y << "\n";
        std::cout << "Distance: " << sqrt( x*x + y*y ) << "\n\n";
        return sqrt( x*x + y*y );
    }

    float calcDirection( const geometry_msgs::Point current_pos, const geometry_msgs::Point desired_pos ){
        float x = desired_pos.x - current_pos.x;
        float y = desired_pos.y - current_pos.y;
        return atan2(y,x);
    }

    float calcOrientation( const geometry_msgs::Point current_pos ){

        past_positions.push_back( current_pos );
        if( past_positions.size() < 2 )     return 0;
        if( past_positions.size() > fs )    past_positions.erase( past_positions.begin() );

        for( int i = 0; i < past_positions.size() - 1; i++ ){

        }
    }

    void sendCommand( float distance, float direction, float orientation ){
        std_msgs::Float32MultiArray cmd;
        float vel_max = 0.3;
        float rot_max = 0.6;
        float slope = 10;

        std::cout << "Direction: " << direction << "\nOrientation: " << -orientation << "\n";
        // if( abs( direction - orientation ) > 2*M_PI ){
        //     direction = copysign( 2*M_PI, orientation ) + ( direction - orientation);
        // } else {
        //     direction = direction - orientation;
        // }
        std::cout << "DIFF: " << direction - orientation << "\n";
        orientation = -orientation;
        float diff = direction - orientation;
        if( abs( diff ) > 2*M_PI ){
            diff = copysign( 2*M_PI, orientation ) + ( direction - orientation);
        }
        if( abs(diff) < M_PI/10){
            direction = 0;
        } else if( diff > 0 ){
            direction = rot_max;
        } else if( diff < 0 ){
            direction = -rot_max;
        }
        cmd.data.push_back( direction );

        if( ( distance - pos_accuracy) > 0 ){
            //cmd.data.push_back( vel_max - ( vel_max / pow(slope,(distance-pos_accuracy)) ) );
            cmd.data.push_back( vel_max );
        } else {
            cmd.data.push_back( 0 );
        }
        command_pub.publish( cmd );
    }

    void sendInfo( float distance, float direction, float orientation ){
        motion_ctrl::DriveInfo info;
        info.direction.data = direction;
        info.distance.data = distance;
        info.orientation.data = orientation;
        info_pub.publish( info );
    }

    void destReached(){
        N++;
        std::cout << "Waypoint reached.\n";
        if( N == waypoint.size() ){
            sleep = true;
        } else {
            sleep = false;
        }
        sendCommand( 0, 0, 0 );
        sendInfo( 0, 0, 0 );
    }

    bool checkGroundBoundaries( point position ){
        point bound1, bound2;
        bound1.x = -1.36;
        bound1.y = -1.8;
        bound2.x = 2;
        bound2.y = 2;
        if( (position.x < bound1.x)||(position.x > bound2.x )||
            (position.y < bound1.y)||(position.y > bound2.y ) ){
            return false;
        }
        return true;
    }

};

class SubAndPub{
private:
    ros::NodeHandle hand;
    MotionControl obj;
    ros::Subscriber waypoint_sub;
    ros::Subscriber sensor_processing_sub;

public:
    SubAndPub(){
        sensor_processing_sub = hand.subscribe( "sensor_processing_data", 20, &SubAndPub::dataCallback, this );
        waypoint_sub = hand.subscribe( "waypoint_data", 100, &SubAndPub::wayCallback, this );
        std::cout << "SubAndPub initialized...\n";
    }
    void dataCallback( const motion_ctrl::SensorProcessingData::ConstPtr& msg ){
        obj.updatePose( msg );
    }
    void wayCallback( const geometry_msgs::Point::ConstPtr& msg ){
        obj.updateWay( msg );
    }
};

int main( int argc, char *argv[]){
    ros::init( argc, argv, "motion_ctrl" );
    SubAndPub obj;
    ros::spin();
    return 0;
}
