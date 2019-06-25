#include "ros/ros.h"
#include "sequitur_pose/SequiturData.h"
#include "motion_ctrl/DriveInfo.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32MultiArray.h"
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <vector>

class MotionControl{
private:
    ros::NodeHandle hand;
    ros::Publisher info_pub;
    ros::Publisher command_pub;
    std::vector<geometry_msgs::Point> waypoint;
    int N;
    bool sleep;
    float pos_accuracy;
    float direction;
    float distance;
public:
    MotionControl(){
        info_pub = hand.advertise<motion_ctrl::DriveInfo>( "drive_info", 10 );
        command_pub = hand.advertise<std_msgs::Float32MultiArray>( "robot1_controller", 10 );
        N = 0;
        sleep = true;
        pos_accuracy = 0.15;
        direction = 0;
        distance = 0;
        std::cout << "Motion Control initialized...\n";
    }

    void updateWay( const geometry_msgs::Point::ConstPtr& new_point ){
        waypoint.push_back( *new_point );
        sleep = false;
    }

    void updatePose( const sequitur_pose::SequiturData::ConstPtr& msg ){
        if( sleep == false ){
            float distance = calcDistance(msg->pose.position, dest());
            float direction = calcDirection(msg->pose.position, dest());

            if ( (distance - pos_accuracy) < 0 ) destReached();
            sendCommand( distance, direction );
            sendInfo( distance, direction);
        }
    }

    float calcDistance( const geometry_msgs::Point current_pos, const geometry_msgs::Point* desired_pos ){
        float x = desired_pos->x - current_pos.x;
        float y = desired_pos->y - current_pos.y;
        return sqrt( x*x + y*y );
    }

    float calcDirection( const geometry_msgs::Point current_pos, const geometry_msgs::Point* desired_pos ){
        float x = desired_pos->x - current_pos.x;
        float y = desired_pos->y - current_pos.y;
        return atan2(y,x);
    }

    void sendCommand( float direction, float distance ){
        std_msgs::Float32MultiArray cmd;
        float vel_max = 0.8;
        float slope = 10;

        cmd.data[0] =  direction / M_PI;

        if( ( distance - pos_accuracy) > 0 ){
            cmd.data[1] = vel_max - ( vel_max / pow(slope,(distance-pos_accuracy)) );
        }
        else {
            cmd.data[1] = 0;
        }

        command_pub.publish( cmd );
    }

    void sendInfo( float direction, float distance ){
        motion_ctrl::DriveInfo info;
        info.direction.data = direction;
        info.distance.data = distance;
        info_pub.publish( info );
    }

    void destReached(){
        N++;
        if( N > waypoint.size() ){
            sleep = true;
        } else {
            sleep = false;
        }
    }

    geometry_msgs::Point* dest(){
        return &waypoint[N];
    }

};

class SubAndPub{
private:
    ros::NodeHandle hand;
    MotionControl obj;
    ros::Subscriber sequitur_sub;
    ros::Subscriber waypoint_sub;

public:
    SubAndPub(){
        sequitur_sub = hand.subscribe( "sequitur_data", 20, &SubAndPub::dataCallback, this );
        waypoint_sub = hand.subscribe( "waypoint_data", 1000, &SubAndPub::wayCallback, this );
        std::cout << "SubAndPub initialized...\n";
    }
    void dataCallback( const sequitur_pose::SequiturData::ConstPtr& msg ){
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
