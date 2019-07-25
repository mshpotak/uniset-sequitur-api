#include "ros/ros.h"
#include "sequitur_pose/SequiturData.h"
#include "geometry_msgs/Point.h"
#include <vector>
#include <iostream>

typedef geometry_msgs::Point point;

class MovingAverage{
private:
    int avgTerms;
    float alpha;
    std::vector<point> data;
    point avgData;
public:
    MovingAverage( int user_avgTerms = 2, float user_alpha = 0.9 ){
        avgTerms = user_avgTerms;
        alpha = user_alpha;
    }

    void update( point new_data ){
        if( data.size() >= avgTerms ){
            data.erase( data.begin() );
        } else if (data.size() < avgTerms ) {
            if( data.empty() ){
                data.push_back( new_data );
                avgData = new_data;
                return;
            }
        }
        avgData.x = alpha*new_data.x + (1-alpha)*avgData.x;
        avgData.y = alpha*new_data.y + (1-alpha)*avgData.y;
        avgData.z = alpha*new_data.z + (1-alpha)*avgData.z;
    }

    point get(){
        return avgData;
    }
};

class SubAndPub{
private:
    ros::NodeHandle hand;
    ros::Subscriber sequitur_sub;
    ros::Publisher movavg_pub;
    MovingAverage movAvg;
public:
    SubAndPub(): movAvg( 4, 0.1 ){
        sequitur_sub = hand.subscribe( "sequitur_data", 20, &SubAndPub::filterCallback, this );
        movavg_pub = hand.advertise<point>( "moving_average_position", 100 );
        std::cout << "Subscribers and Publishers initialized.\n";
    }

    void filterCallback( const sequitur_pose::SequiturData::ConstPtr& msg  ){
        movAvg.update( msg->pose.position );
        movavg_pub.publish( movAvg.get() );
    }
};


int main( int argc, char *argv[]){
    ros::init( argc, argv, "filter_test" );
    std::cout << "Node 'filter_test' started.\n";
    SubAndPub obj;
    ros::spin();
    return 0;
}
