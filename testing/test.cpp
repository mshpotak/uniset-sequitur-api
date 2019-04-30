#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <poll.h>
#include <unistd.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <errno.h>

using namespace std;

int main()
{
    int result;
    struct ifaddrs *addr = NULL;
    struct ifaddrs *ifs = NULL;

    result = getifaddrs(&ifs);
    if( result == -1 ){
        perror();
        return -1;
    }
    addr = ifs;
    while( addr != NULL ){
        printf("%s\n", addr->ifa_name);
        addr = addr->ifa_next;
    }

    freeifaddrs( ifs );
    return 0;
}
