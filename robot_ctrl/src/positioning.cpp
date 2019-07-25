#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "sequitur_pose/SequiturData.h"

#include <fstream>
#include <math.h>
#include <vector>
#include <iostream>

#include "lib-vector.hpp"
#include "position-proc.hpp"

typedef std_msgs::Float32 fl32;
typedef std_msgs::Float32MultiArray fl32array;
typedef geometry_msgs::Point point;
typedef geometry_msgs::Vector3 vector;
typedef const geometry_msgs::Point::ConstPtr& waypointer;
typedef const sequitur_pose::SequiturData::ConstPtr& seqpointer;

DoubleArrayVector ros2dov( point v ){
    DoubleArrayVector result;
    result.x = v.x;
    result.y = v.y;
    result.z = v.z;
    return result;
}
DoubleArrayVector ros2dov( vector v ){
    DoubleArrayVector result;
    result.x = v.x;
    result.y = v.y;
    result.z = v.z;
    return result;
}
point dov2ros( DoubleArrayVector v ){
    point result;
    result.x = v.x;
    result.y = v.y;
    result.z = v.z;
    return result;
}

class GeneralProc{
private:
    std::ofstream ofs_raw_position;
    std::ofstream ofs_raw_gyro;
    std::ofstream ofs_raw_accel;
    std::ofstream ofs_raw_mag;

    PositionProc vx,vy,vz;
    PositionFilter pf;

    void writeRawPositions( point pos ){
        ofs_raw_position << pos.x << " " <<
                            pos.y << " " <<
                            pos.z << "\n";

    }
    void writeRawGyro( vector gyro ){
        ofs_raw_gyro << gyro.x << " " <<
                        gyro.y << " " <<
                        gyro.z << "\n";
    }
    void writeRawAccel( vector accel ){
        ofs_raw_accel << accel.x << " " <<
                         accel.y << " " <<
                         accel.z << "\n";
    }
    void writeRawMag( vector mag ){
        ofs_raw_mag << mag.x << " " <<
                       mag.y << " " <<
                       mag.z << "\n";
    }
    void writeRawData( seqpointer sequitur_data ){
        writeRawPositions( sequitur_data->pose.position );
        writeRawGyro( sequitur_data->accel.angular );
        writeRawAccel( sequitur_data->accel.linear );
        writeRawMag( sequitur_data->magnetic_field );
    }

    ros::NodeHandle hand;
    ros::Publisher vel_pub;

public:
    GeneralProc():  ofs_raw_position("raw_positions.csv", std::ofstream::trunc ),
                    ofs_raw_gyro("raw_gyro.csv", std::ofstream::trunc ),
                    ofs_raw_accel("raw_accel.csv", std::ofstream::trunc ),
                    ofs_raw_mag("raw_mag.csv", std::ofstream::trunc ){
        vel_pub = hand.advertise<point>("velocity_data",100);
    }
    ~GeneralProc(){
        ofs_raw_position.close();
        ofs_raw_gyro.close();
        ofs_raw_accel.close();
        ofs_raw_mag.close();
    }
    void update( seqpointer sequitur_data ){
        writeRawData( sequitur_data );

        double time = (sequitur_data->header.stamp.toNSec()*pow(10,-9));
        vx.update(sequitur_data->pose.position.x, time);
        vy.update(sequitur_data->pose.position.y, time);
        vz.update(sequitur_data->pose.position.z, time);
        point vel;
        vel.x = vx.getVelocity();
        vel.y = vy.getVelocity();
        vel.z = vz.getVelocity();
        vel_pub.publish( vel );
        DoubleArrayVector v;
        v = ros2dov( sequitur_data->pose.position );
        double velocity = sqrt(vx.getVelocity()*vx.getVelocity() + vy.getVelocity()*vy.getVelocity());
        std::cout << "Velocity: " << velocity << "\n";
        velocity = velocity/2.5;
        std::cout << "Normilized velocity: " << velocity << "\n";
        pf.update( &v, velocity );
    }
    void init( seqpointer sequitur_data ){
        double time = (sequitur_data->header.stamp.toNSec()*pow(10,-9));
        vx.init(sequitur_data->pose.position.x, time);
        vy.init(sequitur_data->pose.position.y, time);
        vz.init(sequitur_data->pose.position.z, time);
    }
    DoubleArrayVector getFPosition(){
        return pf.getPosition();
    }
};

class SubAndPub{
private:
    ros::NodeHandle hand;
    ros::Subscriber data_sub;
    ros::Publisher filter_pub;
    GeneralProc tag;
public:
    SubAndPub(){
        data_sub = hand.subscribe( "sequitur_data", 20, &SubAndPub::initCallback, this );
        filter_pub = hand.advertise<point>( "filter_data", 20 );
    }

    void procCallback( seqpointer sequitur_data ){
        tag.update( sequitur_data );
        filter_pub.publish( dov2ros( tag.getFPosition() ) );
    }

    void initCallback( seqpointer sequitur_data ){
        tag.init( sequitur_data );
        data_sub = hand.subscribe( "sequitur_data", 20, &SubAndPub::procCallback, this );
    }

};

int main( int argc, char *argv[]){
    ros::init( argc, argv, "positioning" );
    std::cout << "Node started.\n";
    SubAndPub obj;
    ros::spin();
    return 0;
}
