#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>

int main( int argc, char *argv[]){
    ros::init( argc, argv, "uniseq_pose");
    ros::NodeHandle hand;
    ros::Publisher pose_pub = hand.advertise<geometry_msgs::PoseStamped>("pose",1000);
    ros::Rate loop_rate(10);
    int count = 0;
    // estabilish connection
    while(ros::ok()){
        geometry_msgs::PoseStamped msg;

        //send msg
        //receive msg
        msg.header.seq = count;
        msg.pose.position.x = 0;
        msg.pose.position.y = 0;
        msg.pose.position.z = 0;

        ROS_INFO("count = %d:\tx = %f\ty = %f\tz = %f", msg.header.seq, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z );
        pose_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
