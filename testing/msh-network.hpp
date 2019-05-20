#include <string>
#include <cstring>
#include <stdio.h>
#include <sys/socket.h>
#include <netdb.h>
#include <poll.h>
#include <unistd.h>

#ifndef MSHNETWORK_HPP_
#define MSHNETWORK_HPP_

class Network{
    protected:
        Network();
        int sock_fd;
        int result;
        std::string msg;

        int connect_to( const char ip[], const char port[], int family = AF_INET, int socktype = SOCK_DGRAM, int flags = AI_PASSIVE );
        int send_msg( const char* msg );
        int recv_msg();
        int await( int timeout_ms = 1000);
};

#endif
