#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "sensor_msgs/MagneticField.h"
#include "seq-req-class-v2.hpp"
#include "attitude_estimator.h"
#include <math.h>

using namespace stateestimation;

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
    ros::init( argc, argv, "uniseq_pose");
    ros::NodeHandle hand;
    ros::Publisher pose_pub = hand.advertise<geometry_msgs::PoseStamped>("sequitur_pose",1000);
    ros::Publisher accel_pub = hand.advertise<geometry_msgs::AccelStamped>("sequitur_accel",1000);
    ros::Publisher mag_pub = hand.advertise<sensor_msgs::MagneticField>("sequitur_mag",1000);
    int count = 0;
    Sequitur seq;
    AttitudeEstimator est;
    geometry_msgs::PoseStamped msg_pose;
    geometry_msgs::AccelStamped msg_accel;
    sensor_msgs::MagneticField msg_mag;
    double q[4];
    double dt = 0;
    seq.anchor.set_loc();
    seq.tag.recv_upd();
    msg_conv( seq.tag.pose, &msg_pose, &msg_accel, &msg_mag );
    ros::Time timestamp_prev = msg_pose.header.stamp;

    while( ros::ok() ){

        seq.tag.recv_upd();
        msg_pose.header.seq = count++;
        msg_conv( seq.tag.pose, &msg_pose, &msg_accel, &msg_mag );
        dt = msg_pose.header.stamp.toSec() - timestamp_prev.toSec();
        est.update( dt,
                    msg_accel.accel.angular.x, msg_accel.accel.angular.y, msg_accel.accel.angular.z,
                    msg_accel.accel.linear.x, msg_accel.accel.linear.y, msg_accel.accel.linear.z,
                    msg_mag.magnetic_field.x, msg_mag.magnetic_field.y, msg_mag.magnetic_field.z );
        est.getAttitude( q );
        upd_quaterion( q, &msg_pose );
        pose_pub.publish(msg_pose);
        accel_pub.publish(msg_accel);
        mag_pub.publish(msg_mag);
        timestamp_prev = msg_pose.header.stamp;
        ros::spinOnce();
    }

    return 0;
}
