#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include <experimental/filesystem>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

typedef geometry_msgs::Point point;

bool fexists( std::string fname ){
    std::ifstream ifs(fname.c_str());
    return ifs.good();
}

class MapControl{
private:
    ros::NodeHandle hand;
    ros::Publisher waypoint_pub;
    std::vector<point> list;
    point ltop, lbot;
    point rtop, rbot;
    point bound1, bound2;

public:
    MapControl(){
        waypoint_pub = hand.advertise<geometry_msgs::Point>( "waypoint_data", 100 );
        std::cout << "Map Control initialized...\n";
        if( fexists("map.csv") ){
            parseList( "map.csv", 4 );
            lbot = list[0];
            ltop = list[1];
            rbot = list[2];
            rtop = list[3];
            std::cout << "Reading 'map.csv' file...\n";
        } else {
            setDefaultMap();
            std::cout << "Setting default map settings...\n";
        }
        printPoint( "LEFT BOT", lbot );
        printPoint( "LEFT TOP", ltop );
        printPoint( "RIGHT BOT", rbot );
        printPoint( "RIGHT TOP", rtop );

        float safe_coef = 0.8;
        std::cout << "\nSetting boundaries...\n";
        std::cout << "Safe coefficient: [" << safe_coef << "]...\n";
        setBoundaries( safe_coef );
        printPoint( "BOT ", bound1 );
        printPoint( "TOP ", bound2 );
        std::cout << "Box size: X[ " << bound2.x-bound1.x << " ] Y[ " << bound2.y-bound1.y << " ]\n";
    }

    void setDefaultMap(){
        lbot.x = -1.856; lbot.y = -2.247; lbot.z = 2.095;
        ltop.x = -1.548; ltop.y = 2.541;  ltop.z = 0.705;
        rbot.x = 2.439;  rbot.y = -2.231; rbot.z = 0.695;
        rtop.x = 2.631;  rtop.y = 2.544;  rtop.z = 2.145;
    }

    int parseList( std::string filename, int n_points = 1, int position = 0 ){
        std::ifstream ifs( filename.c_str(), std::ifstream::in );
        std::stringstream ss;
        std::string data;
        point point;

        ifs.seekg(position);
        for( int i = 0; i < n_points; i++ ){
            getline( ifs, data );
            ss << data;
            ss >> point.x >> point.y >> point.z;
            data.clear();
            std::stringstream().swap(ss);
            list.push_back(point);
            std::cout << list[i] << "\n";
        }

        position = ifs.tellg();
        ifs.close();
        return position;
    }

    void setBoundaries( float safe_coef ){
        bound1.x = safe_coef*(lbot.x + ltop.x)/2;
        bound1.y = safe_coef*(lbot.y + rbot.y)/2;
        bound1.z = safe_coef*(ltop.z + rbot.z)/2;

        bound2.x = safe_coef*(rtop.x + rbot.x)/2;
        bound2.y = safe_coef*(rtop.y + ltop.y)/2;
        bound2.z = safe_coef*(rtop.z + lbot.z)/2;
    }

    void printPoint( std::string pname, point point ){
        std::cout << pname << ": "  << point.x << " " << point.y << " " << point.z << "\n";
    }

    void inputWaypoint(){
        point point;
        std::cout << "\nInput next waypoint:\n";
        std::cout << "x: ";
        std::cin >> point.x;
        std::cout << "y: ";
        std::cin >> point.y;
        point.z = 0;
        point = checkBoundaries( &point );
        waypoint_pub.publish( point );
    }

    point checkBoundaries( point *point ){
        if( point->x < bound1.x ){
            point->x = bound1.x;
        } else if( point->x > bound2.x){
            point->x = bound2.x;
        }
        if( point->y < bound1.y ){
            point->y = bound1.y;
        } else if( point->y > bound2.y){
            point->y = bound2.y;
        }

        return *point;
    }
};

int main( int argc, char *argv[]){
    ros::init( argc, argv, "map_ctrl" );
    std::cout << "Node started...\n";
    MapControl obj;
    while( ros::ok() ){
        obj.inputWaypoint();
        ros::spinOnce();
    };
    return 0;
}
