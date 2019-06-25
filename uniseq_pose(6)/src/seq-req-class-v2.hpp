#include <string>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <sys/socket.h>
#include <netdb.h>
#include <poll.h>
#include <unistd.h>
#include <math.h>

#ifndef SEQREQCLASSV2_HPP_
#define SEQREQCLASSV2_HPP_

#define CLIENT_PING 0
#define CLIENT_SET_PARAMETER 3
#define CLIENT_GET_SENSORS 51
#define CLIENT_GET_RANGE 50
#define CLIENT_SCAN 56
#define CLIENT_SEND_DATA 57
#define CLIENT_GET_ANCHOR_INFO 10
#define CLIENT_SET_ANCHOR_INFO 11
#define CLIENT_GET_TAG_POSITION 14
#define CLIENT_POSITION_FORWARD 59

#define BUFF_SIZE 1024
#define PORT_SEQ  "5678"

struct xyz{
    double x;
    double y;
    double z;
};

struct imu{
    double timestamp;
    struct xyz accel;
    struct xyz gyro;
    struct xyz mag;
};

struct pose{
    double timestamp;
    struct xyz position;
    struct xyz conf;
};

struct pose_with_imu{
    double timestamp;
    struct xyz position;
    struct xyz posconf;
    struct xyz accel;
    struct xyz gyro;
    struct xyz mag;
};

class Network{
    private:
        int connect_to( const char ip[], const char port[], int family = AF_INET, int socktype = SOCK_DGRAM, int flags = AI_PASSIVE );
    protected:
        Network();
        ~Network();
        int result;
        char buffer[BUFF_SIZE];
    public:
        int sock_fd;
        int send_msg( const char* msg );
        int recv_msg();
        int await( int timeout_ms = 1000);
};

class Sequitur: public Network{
    protected:
        int req_code;
        std::string req_parameters;
        int hashcode;
        std::string msg;
    public:
        Sequitur();
        ~Sequitur();
        void compose_msg();
        virtual void decompose_msg();
        void send_req();
        void recv_resp();
        void req_once();
        void set_req( int user_code, std::string user_parameters = " " );
        class ForwardData{
            private:
                Sequitur *seq;
            public:
                ForwardData( Sequitur *owner );
                ~ForwardData();
                pose_with_imu pose;
                void decompose_msg();
                void state( bool var );
                void recv_upd();
        } tag;
        class SetAnchorLocation{
            private:
                Sequitur *seq;
            public:
                SetAnchorLocation( Sequitur *owner );
                std::string parameters[4];
                xyz location;
                void decompose_msg();
                void set_loc();
        } anchor;
        class Scan{
            private:
                Sequitur *seq;
            public:
                double range;
                Scan( Sequitur *owner );
                void decompose_msg();
                double get_range( std::string node_id );
        } scan;
};

#endif
