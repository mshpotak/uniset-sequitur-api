#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "sensor_msgs/MagneticField.h"

#include <fstream>

std::ofstream pos_ofs("pos_out.csv", std::ofstream::trunc);
std::ofstream acc_ofs("acc_out.csv", std::ofstream::trunc);
std::ofstream mag_ofs("mag_out.csv", std::ofstream::trunc);

void poseCallback( const geometry_msgs::PoseStamped::ConstPtr& msg ){
    ROS_INFO( "[count = %d:\tx = %f\ty = %f\tz = %f]",
    msg->header.seq,
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z );

    pos_ofs << msg->header.stamp << " " <<
    msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z << "\n";
}

void accelCallback( const geometry_msgs::AccelStamped::ConstPtr& msg ){
    acc_ofs << msg->header.stamp << " " <<
    msg->accel.linear.x << " " << msg->accel.linear.y << " " << msg->accel.linear.z << " " <<
    msg->accel.angular.x << " " << msg->accel.angular.y << " " << msg->accel.angular.z << "\n";
}

void magCallback( const sensor_msgs::MagneticField::ConstPtr& msg ){
    mag_ofs << msg->header.stamp << " " <<
    msg->magnetic_field.x << " " << msg->magnetic_field.y << " " << msg->magnetic_field.z << "\n";
}

int main( int argc, char *argv[] ){
    ros::init( argc, argv, "uniseq_listen" );
    ros::NodeHandle hand;
    ros::Subscriber pose_sub = hand.subscribe( "sequitur_pose", 1000, poseCallback );
    ros::Subscriber accel_sub = hand.subscribe( "sequitur_accel", 1000, accelCallback );
    ros::Subscriber mag_sub = hand.subscribe( "sequitur_mag", 1000, magCallback );
    ros::spin();
    return 0;
}
