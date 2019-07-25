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

class ROSPositionDriver{
public:
    ROSPositionDriver(){}

    DoubleArrayVector decode( seqpointer msg ){
        DoubleArrayVector v;
        v = ros2dov( msg->pose.position );
        v.z = 0;
        return v;
    };

    DoubleArrayVector decode( point msg ){
        DoubleArrayVector v;
        v = ros2dov( msg );
        v.z = 0;
        return v;
    };

    fl32array encode( float linear_velocity, float angular_velocity ){
        fl32array cmd;
        cmd.data.push_back( angular_velocity );
        cmd.data.push_back( linear_velocity );
        return cmd;
    };
};

class WaypointController{
private:
    std::ofstream ofs_position;
    std::ofstream ofs_velocity;
    std::ofstream ofs_distance;
    std::ofstream ofs_direction;
    std::ofstream ofs_orientation;

    float linear_velocity;
    float angular_velocity;

    float target_radius;
    float position_accuracy;

    float max_velocity;
    float min_velocity;
    float slope;
    float distance;
    float travel_distance;

    float current_direction;
    float current_orientation;
    float previous_orientation;
    std::vector<float> orientations;
    DoubleArrayVector previous_orientational_position;
    DoubleArrayVector previous_position;
    DoubleArrayVector current_position;
    DoubleArrayVector target_position;
    bool target_reached;
    bool target_void;

    ros::NodeHandle hand;
    ros::Publisher orientation_data;
    ros::Publisher direction_data;
    ros::Publisher rotation_data;
    ROSPositionDriver driver;

    float detYaw( DoubleArrayVector position1, DoubleArrayVector position2 ){
        DoubleArrayVector v; //temporary vector
        v = position2 - position1;
        return atan2( v.y, v.x );
    }

    double convTwoPi( double angle ){
        return ((angle>=0)&&(angle<=M_PI))*angle + ((angle<0)&&(angle>=-M_PI))*(2*M_PI+angle);
    }

    double detDistance(){
        DoubleArrayVector v; //temporary vector
        v = target_position - current_position;
        return v.magnitude();
    }

    double detTravelDistance(){
        DoubleArrayVector v; //temporary vector
        v = current_position - previous_position;
        return v.magnitude();
    }

    double detOrientation(){
        // DoubleArrayVector v; //temporary vector
        // v = current_position - previous_position;
        // std::cout << "Magnitude: " << v.magnitude() << "\n";
        // std::cout << "X: " << v.x << "\n";
        // std::cout << "Y: " << v.y << "\n";
        // std::cout << "Z: " << v.z << "\n";

        travel_distance = travel_distance + detTravelDistance();
        if ( travel_distance > position_accuracy ){
            float result = detYaw( previous_orientational_position, current_position );
            previous_orientation = result;
            previous_position = current_position;
            previous_orientational_position = current_position;
            travel_distance = 0;
            //std::cout << "Prev. orientation 2: " << previous_orientation << "\n";
            return result;
        } else {
            previous_position = current_position;
            //std::cout << "Prev. orientation 1: " << previous_orientation << "\n";
            return previous_orientation;
        }
    }

    double detDirection(){
        return detYaw( current_position, target_position );
    }

    float detLinearVelocity(){
        distance = detDistance();
        // std::cout << "Distance: " << distance << "\n";
        return max_velocity - ( max_velocity / (pow( slope, distance - position_accuracy ) ) ) + min_velocity;
    }

    float detAngularVelocity(){
        current_orientation = convTwoPi( detOrientation() );
        current_direction = convTwoPi( detDirection() );
        float diff = current_direction - current_orientation;
        if( diff > M_PI ){
            diff  = diff - 2*M_PI;
        }
        else if( diff < -M_PI ){
            diff = diff + 2*M_PI;
        }
        float rel_diff = diff/M_PI;
        float result = rel_diff;
        //float result = diff*diff*diff/(M_PI*M_PI*M_PI);
                                fl32 data1;
                                data1.data = current_orientation;
                                orientation_data.publish( data1 );
                                fl32 data2;
                                data2.data = current_direction;
                                direction_data.publish( data2 );
                                fl32 data3;
                                data3.data = diff;
                                rotation_data.publish( data3 );
        float w = linear_velocity;
        result = copysign(((1-max_velocity)/max_velocity)*linear_velocity, diff);
        // if( w < 0.2 ){
        //     result = copysign( (1-w) , diff );
        // }else {
        //     result = copysign( w*2 , diff );
        // }
        if(abs(rel_diff) < 0.15 ){
            result = 0;
        }
        return result;
    }

    void control(){
        if( !target_void ){
            if( detDistance() < target_radius ){
                target_reached = true;
                linear_velocity = 0;
                angular_velocity = 0;
            } else {
                target_reached = false;
                linear_velocity = detLinearVelocity();
                angular_velocity = detAngularVelocity();
            }
        }
    }


public:
    WaypointController():
    ofs_position("saved_positions.csv", std::ofstream::trunc ),
    ofs_velocity("saved_velocities.csv", std::ofstream::trunc ),
    ofs_distance("saved_distances.csv", std::ofstream::trunc ),
    ofs_direction("saved_directions.csv", std::ofstream::trunc ),
    ofs_orientation("saved_orientations.csv", std::ofstream::trunc )
    {
        linear_velocity = 0;
        angular_velocity = 0;
        target_reached = true;
        target_void = true;
        position_accuracy = 0.15;
        target_radius = 0.3;
        max_velocity = 0.3;
        min_velocity = 0.1;
        slope = 6;
        distance = 0;
        travel_distance = 0;
        previous_orientation = 0;

        orientation_data = hand.advertise<fl32>("orientation_data", 100);
        direction_data = hand.advertise<fl32>("direction_data", 100);
        rotation_data = hand.advertise<fl32>("rotation_data", 100);
    }

    ~WaypointController(){
        ofs_position.close();
        ofs_velocity.close();
        ofs_distance.close();
        ofs_direction.close();
        ofs_orientation.close();
    }

    void test( seqpointer msg ){
        current_position = driver.decode( msg );
        current_orientation = convTwoPi( detOrientation() );
        fl32 odata;
        odata.data = current_orientation;
        orientation_data.publish( odata );
        linear_velocity = 0.4;
        angular_velocity = -1;
    }

    void update( seqpointer msg ){
        current_position = driver.decode( msg );
        control();
        ofs_position << current_position.x << " " << current_position.y << "\n";
        ofs_velocity << linear_velocity << "\n";
        ofs_distance << distance << "\n";
        ofs_direction << current_direction << "\n";
        ofs_orientation << current_orientation << "\n";

    }

    void update( DoubleArrayVector new_position ){
        current_position = new_position;
        control();
        ofs_position << current_position.x << " " << current_position.y << "\n";
        ofs_velocity << linear_velocity << "\n";
        ofs_distance << distance << "\n";
        ofs_direction << current_direction << "\n";
        ofs_orientation << current_orientation << "\n";

    }

    void setNewTarget( point msg ){
        target_position = driver.decode( msg );
        target_void = false;
        control();
    }

    fl32array getCommand(){
        return driver.encode( linear_velocity, angular_velocity );
    }

    bool isTargetReached(){
        return target_reached;
    }

    bool isTargetVoid(){
        return target_void;
    }

    bool setTargetVoid(){
        target_void = true;
    }
};

class WayPlanner{
private:
    std::ofstream ofs;
    std::vector<point> way;
    int current_target_number;
public:
    WayPlanner(): ofs( "saved_waypoints.csv", std::ofstream::trunc ){
        current_target_number = 0;
        way.clear();
    }

    ~WayPlanner(){
        ofs.close();
    }

    void update( waypointer new_waypoint ){
        way.push_back( *new_waypoint );
        std::cout << "New waypoint!\n";
        ofs << new_waypoint->x << " " << new_waypoint->y << " " << new_waypoint->z << "\n";
    };

    void setTargetReached(){
        current_target_number++;
    }

    bool isTargetPresent(){
        if( way.empty() ){
            return false;
        } else if( current_target_number < way.size() ){
            return true;
        } else {
            return false;
        }
    }

    point getCurrentTarget(){
        return way[current_target_number];
    }

};

class SubAndPub{
private:
    ros::NodeHandle hand;
    ros::Subscriber data_sub;
    ros::Subscriber map_sub;
    ros::Publisher command_pub;
    ros::Publisher filter_pub;
    WaypointController robot1;
    WayPlanner way1;
    PositionFilter pf1;
public:
    SubAndPub(){
        data_sub = hand.subscribe( "sequitur_data", 20, &SubAndPub::robotCtrlCallback, this );
        map_sub = hand.subscribe( "waypoint_data", 20, &SubAndPub::wayCtrlCallback, this );
        command_pub = hand.advertise<fl32array>( "robot1_commander", 20 );
        filter_pub = hand.advertise<point>( "filter_data", 20 );
    }

    void robotCtrlCallback( seqpointer position_data ){
        DoubleArrayVector v = ros2dov(position_data->pose.position);
        fl32array rctrl = robot1.getCommand();
        pf1.update( &v, rctrl.data[1] );
        filter_pub.publish( dov2ros( pf1.getPosition() ) );

        robot1.update( v );
        if( robot1.isTargetReached() ){
            if( !robot1.isTargetVoid() ){
                std::cout << "Void check reached.\n";
                way1.setTargetReached();
                robot1.setTargetVoid();
            }
            if( way1.isTargetPresent() ){
                std::cout << "Waypoint check reached.\n";
                robot1.setNewTarget( way1.getCurrentTarget() );
            }
        }
        command_pub.publish( robot1.getCommand() );
    }

    void robotTestCallback( seqpointer position_data ){
        robot1.test( position_data );
        command_pub.publish( robot1.getCommand() );
    }

    void wayCtrlCallback( waypointer waypoint_data ){
        way1.update( waypoint_data );
    }

};

int main( int argc, char *argv[]){
    ros::init( argc, argv, "robot_ctrl" );
    std::cout << "Node started.\n";
    SubAndPub obj;
    ros::spin();
    return 0;
}
