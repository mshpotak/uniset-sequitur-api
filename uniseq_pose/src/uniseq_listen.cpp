#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

void poseCallback( const geometry_msgs::PoseStamped::ConstPtr& msg ){
    ROS_INFO( "[count = %d:\tx = %f\ty = %f\tz = %f]",
    msg->header.seq,
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z );
}

int main( int argc, char *argv[] ){
    ros::init( argc, argv, "uniseq_listen" );
    ros::NodeHandle hand;
    ros::Subscriber pose_sub = hand.subscribe( "pose", 1000, poseCallback );
    ros::spin();
    return 0;
}
