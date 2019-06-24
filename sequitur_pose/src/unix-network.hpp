#include <string>
#include <cstring>
#include <stdio.h>
#include <sys/socket.h>
#include <netdb.h>
#include <poll.h>
#include <unistd.h>

#ifndef UNIXNETWORK_HPP_
#define UNIXNETWORK_HPP_

#define BUFF_SIZE 512

class Network{
    protected:
        int sock_fd;

    public:
        Network();
        int result;
        std::string msg;

        int connect_to( const char ip[], const char port[], int family = AF_INET, int socktype = SOCK_DGRAM, int flags = AI_PASSIVE );
        int send_msg( const char* msg );
        int recv_msg();
        int await( int timeout_ms = 1000);
};

#endif
