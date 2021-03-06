#include <string>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "unix-network.hpp"

#ifndef SEQUITURAPI_HPP_
#define SEQUITURAPI_HPP_

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

class Sequitur: public Network{
    private:
        int hashcode;
        int req_code;
        std::string req_parameters;

        void set_network();
    public:
        Sequitur();
        ~Sequitur();

        void compose_msg();
        virtual void decompose_msg();
        void send_req();
        void recv_resp();
        void req_once();
        void set_req( int user_code, std::string user_parameters = " " );
        void print_msg();

        class Scan{
            private:
                Sequitur *seq;
            public:
                Scan( Sequitur *owner );

                double range;
                double get_range( std::string node_id );
                void decompose_msg();
        };
};

class Anchor: public Sequitur{};

class Tag: public Sequitur{
    public:
        class ForwardData{
            private:
                Sequitur *seq;
            public:
                ForwardData( Sequitur *owner );
                ~ForwardData();

                pose_with_imu pose;
                void fwd_state( bool fwd );
                void recv_upd( bool print = false );
                void decompose_msg();
        };
        class SetAnchorLocation{
            private:
                Sequitur *seq;
            public:
                SetAnchorLocation( Sequitur *owner );
                std::string parameters[4];

                void set_default_loc();
                void decompose_msg();
        };
};

#endif
