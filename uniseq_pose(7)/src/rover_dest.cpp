#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <limits.h>

using namespace std;

int main( int argc, char *argv[]){
    ros::init( argc, argv, "rover_dest");
    ros::NodeHandle hand;
    ros::Publisher pose_pub = hand.advertise<geometry_msgs::PoseStamped>("rover_destination",1000);
    int count = 0;
    geometry_msgs::PoseStamped msg_pose;

    while( ros::ok() ){
        cout << "Enter [x y] coordinate:\t";
        cin >> msg_pose.pose.position.x;
        cin >> msg_pose.pose.position.y;
        cout << "\n";
        cin.clear();

        pose_pub.publish(msg_pose);
        count++;
        ros::spinOnce();
    }

    return 0;
}
