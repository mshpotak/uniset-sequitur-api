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

using namespace std;

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
    return 0;
}
