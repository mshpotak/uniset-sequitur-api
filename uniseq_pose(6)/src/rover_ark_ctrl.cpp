#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"



class SubAndPub{
    public:
        SubAndPub(){
            ark_pub = hand.advertise<std_msgs::Int8>( "rover_command", 200 );
            ark_sub = hand.subscribe( "key_vel", 200, &SubAndPub::arkCallback, this );
    }

        void arkCallback( const geometry_msgs::Twist::ConstPtr& msg ){
            std_msgs::Int8 fwd; fwd.data = 1;
            std_msgs::Int8 bwd; bwd.data = 2;
            std_msgs::Int8 stop; stop.data = 0;
            std_msgs::Int8 left; left.data = 3;
            std_msgs::Int8 right; right.data = 4;
            if( msg->angular.z > 0 ) {
                ark_pub.publish( left );
            } else if( msg->angular.z < 0 ) {
                ark_pub.publish( right );
            } else if( msg->angular.z == 0 ){

                if ( msg->linear.x > 0 ) {
                    ark_pub.publish( fwd );
                } else if ( msg->linear.x < 0 ) {
                    ark_pub.publish( bwd );
                } else ark_pub.publish( stop );

            }
        }

    private:
        ros::NodeHandle hand;
        ros::Publisher ark_pub;
        ros::Subscriber ark_sub;
};

int main( int argc, char *argv[] ){
    ros::init( argc, argv, "rover_ark_ctrl" );
    SubAndPub ark_obj;
    ros::spin();
    return 0;
}
