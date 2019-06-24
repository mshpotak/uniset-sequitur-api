#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "sensor_msgs/MagneticField.h"
#include "sequitur_pose/SequiturData.h"
#include "sequitur-api.hpp"
#include <math.h>

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
    c->accel.angular.x = a.gyro.x*M_PI/180;
    c->accel.angular.y = a.gyro.y*M_PI/180;
    c->accel.angular.z = a.gyro.z*M_PI/180;

    d->magnetic_field.x = a.mag.x;
    d->magnetic_field.y = a.mag.y;
    d->magnetic_field.z = a.mag.z;
}

void upd_quaterion( double *a, geometry_msgs::PoseStamped *b){
    b->pose.orientation.x = a[0];
    b->pose.orientation.y = a[1];
    b->pose.orientation.z = a[2];
    b->pose.orientation.w = a[3];
}

int main( int argc, char *argv[]){
    ros::init( argc, argv, "sequitur_pose");
    ros::NodeHandle hand;
    ros::Publisher sequitur_pub = hand.advertise<sequitur_pose::SequiturData>("sequitur_data",1000);

    int count = 0;
    Tag tag0;
    Tag::ForwardData tag0f0 = &tag0;
    Tag::SetAnchorLocation tag0f1 = &tag0;
    sequitur_pose::SequiturData msg_seq;
    geometry_msgs::PoseStamped msg_pose;
    geometry_msgs::AccelStamped msg_accel;
    sensor_msgs::MagneticField msg_mag;
    tag0f1.set_default_loc();
    tag0f0.fwd_state( true );
    tag0f0.recv_upd();

    while( ros::ok() ){
        tag0f0.recv_upd();
        msg_conv( tag0f0.pose, &msg_pose, &msg_accel, &msg_mag );

        msg_seq.header.seq = count++;
        msg_seq.header.stamp = msg_pose.header.stamp;
        msg_seq.pose = msg_pose.pose;
        msg_seq.accel = msg_accel.accel;
        msg_seq.magnetic_field = msg_mag.magnetic_field;

        sequitur_pub.publish(msg_seq);
        ros::spinOnce();
    }

    return 0;
}
