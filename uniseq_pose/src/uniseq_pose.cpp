#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "sensor_msgs/MagneticField.h"
#include "seq-req-class-v2.hpp"

void msg_conv( pose_with_imu a, geometry_msgs::PoseStamped *b, geometry_msgs::AccelStamped *c, sensor_msgs::MagneticField *d){
    ros::Time stamp(a.timestamp/1000);
    b->header.stamp = stamp;
    c->header.stamp = stamp;
    d->header.stamp = stamp;

    b->pose.position.x = a.position.x;
    b->pose.position.y = a.position.y;
    b->pose.position.z = a.position.z;

    c->accel.linear.x = a.accel.x;
    c->accel.linear.y = a.accel.y;
    c->accel.linear.z = a.accel.z;
    c->accel.angular.x = a.gyro.x;
    c->accel.angular.y = a.gyro.y;
    c->accel.angular.z = a.gyro.z;

    d->magnetic_field.x = a.mag.x;
    d->magnetic_field.y = a.mag.y;
    d->magnetic_field.z = a.mag.z;
}

int main( int argc, char *argv[]){
    ros::init( argc, argv, "uniseq_pose");
    ros::NodeHandle hand;
    ros::Publisher pose_pub = hand.advertise<geometry_msgs::PoseStamped>("sequitur_pose",1000);
    ros::Publisher accel_pub = hand.advertise<geometry_msgs::AccelStamped>("sequitur_accel",1000);
    ros::Publisher mag_pub = hand.advertise<sensor_msgs::MagneticField>("sequitur_mag",1000);
    int count = 0;
    Forward tag;
    geometry_msgs::PoseStamped msg_pose;
    geometry_msgs::AccelStamped msg_accel;
    sensor_msgs::MagneticField msg_mag;

    while( ros::ok() ){

        tag.recv_upd();
        msg_pose.header.seq = count++;
        msg_conv( tag.pose, &msg_pose, &msg_accel, &msg_mag );

        pose_pub.publish(msg_pose);
        accel_pub.publish(msg_accel);
        mag_pub.publish(msg_mag);
        ros::spinOnce();
    }

    return 0;
}
