#include "ros/ros.h"
#include "sequitur_pose/SequiturData.h"
#include "motion_ctrl/DriveInfo.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32MultiArray.h"
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
    std::ifstream ifs;
    std::vector<point> list;
    point ltop, lbot;
    point rtop, rbot;

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
        std::cout << "LEFT BOT: "  << lbot.x << " " << lbot.y << " " << lbot.z << "\n";
        std::cout << "LEFT TOP: "  << ltop.x << " " << ltop.y << " " << ltop.z << "\n";
        std::cout << "RIGHT BOT: " << rbot.x << " " << rbot.y << " " << rbot.z << "\n";
        std::cout << "RIGHT TOP: " << rtop.x << " " << rtop.y << " " << rtop.z << "\n";
    }

    void setDefaultMap(){
        lbot.x = -1.856; lbot.y = -2.247; lbot.z = 2.095;
        ltop.x = -1.548; ltop.y = 2.541;  ltop.z = 0.705;
        rbot.x = 2.439;  rbot.y = -2.231; rbot.z = 0.695;
        rtop.x = 2.631;  rtop.y = 2.544;  rtop.z = 2.145;
    }

    int parseList( std::string filename, int n_points = 1, int position = 0 ){
        ifs.open( filename.c_str(), std::ifstream::in );
        std::stringstream ss;
        char data[100];
        point point;

        ifs.seekg(position);
        for( int i = 0; i < n_points; i++ ){
            ifs.getline( data, 100 );
            ss << data;
            ss >> point.x >> point.y >> point.z;
            ss.flush();
            memset( data, 0, 100);
            list.push_back(point);
        }
        position = ifs.tellg();
        ifs.close();
        return position;
    }
};

int main( int argc, char *argv[]){
    ros::init( argc, argv, "map_ctrl" );
    std::cout << "Node started...\n";
    MapControl obj;
    while( ros::ok() ){
        ros::spinOnce();
    };
    return 0;
}
