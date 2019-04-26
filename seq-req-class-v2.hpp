#include <string>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <netdb.h>

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

class Sequitur{
    private:
        int sock_fd;
        std::string parameters;


    public:
        Sequitur();
        ~Sequitur();

        void connect_to(const char ip[],
                        const char port[],
                        int family = AF_INET,
                        int socktype = SOCK_DGRAM,
                        int flags = AI_PASSIVE);

        void send_request();
        void recv_reply();

        struct reply{
            double timestamp;

            struct accel{
                double x;
                double y;
                double z;
            } accel;

            struct gyro{
                double x;
                double y;
                double z;
            } gyro;

            struct mag{
                double x;
                double y;
                double z;
            } mag;

        };
}



class Seq_Request{
    private:
        int code;
        std::string parameters;
        int hashcode;
        const char* send_msg(int req_hashcode, int req_code, std::string req_parameters);
        std::string recv_msg(const char* msg);
    public:
        Seq_Request(int user_code);
        void send_request();
        void recv_response();
        void rec_response();
        int check_hashcode();

};

#endif
