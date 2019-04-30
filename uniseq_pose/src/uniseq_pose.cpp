#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "seq-req-class-v2.hpp"

#include <sstream>

void msg_conv( pose_stamped a, geometry_msgs::PoseStamped b){
    b.pose.position.x = a.position.x;
    b.pose.position.y = a.position.y;
    b.pose.position.z = a.position.z;
    ros::Time stamp(a.timestamp/1000);
    b.header.stamp = stamp;
}

int main( int argc, char *argv[]){
    ros::init( argc, argv, "uniseq_pose");
    ros::NodeHandle hand;
    ros::Publisher pose_pub = hand.advertise<geometry_msgs::PoseStamped>("pose",1000);
    ros::Rate loop_rate(10);
    int count = 0;
    Forward fwd;
    geometry_msgs::PoseStamped msg;
    while( ros::ok() ){

        fwd.recv_upd();
        msg.header.seq = count;
        msg_conv( fwd.pose, msg );

        ROS_INFO("count = %d:\tx = %f\ty = %f\tz = %f", msg.header.seq, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z );
        pose_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
