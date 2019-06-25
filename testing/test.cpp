<<<<<<< HEAD
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <poll.h>
#include <unistd.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <errno.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
=======
>>>>>>> 9c87d3a1626395d62451eadc73e1054630850a8e

#include "sequitur-api.hpp"
#include <math.h>
#include <chrono>
#include <ctime>

<<<<<<< HEAD
int main()
{
    int result;
    int i;
    struct ifaddrs *addr = NULL;
    struct ifaddrs *ifs = NULL;
    char buff[INET_ADDRSTRLEN];

    result = getifaddrs(&ifs);
    if( result == -1 ){
        perror("getifaddrs error");
        return -1;
    }
    addr = ifs;
    while( addr != NULL ){
        printf("\n%s\t", addr->ifa_name);
        if(addr->ifa_addr->sa_family == AF_INET){
            inet_ntop( AF_INET, &(((struct sockaddr_in*)(addr->ifa_addr))->sin_addr.s_addr), buff, INET_ADDRSTRLEN);
            printf("%s\t", buff);
            if( strstr(buff, "192.168") ){
                printf("Found it!\t");
                result = socket( AF_INET, SOCK_DGRAM, AI_PASSIVE );
                if( result == -1){
                    perror("socket error");
                    return -1;
                }
                for( i = 0; i <= 255; i++){
                    connect()

                }
            }
        }
        addr = addr->ifa_next;
    }


    freeifaddrs( ifs );
=======
int main( int argc, char *argv[]){
    printf("Start...\n");
    Sequitur seq;
    std::chrono::system_clock::time_point start, end;
    std::chrono::duration<double> t_cycle;
    unsigned int i = 1;
    double t_sum = 0, range_avg = 0;
    while(1){
        start = std::chrono::system_clock::now();
        seq.scan.get_range("10205F1310001425");
        end = std::chrono::system_clock::now();
        t_cycle = end-start;
        range_avg += seq.scan.range;
        t_sum += t_cycle.count();
        printf("Range: %f, Range avg: %f, \tTime: %f,\t Frequency: %f\n", seq.scan.range, range_avg/i, t_cycle.count(), 1/(t_sum/i) );
        i++;
    }
>>>>>>> 9c87d3a1626395d62451eadc73e1054630850a8e
    return 0;
}
