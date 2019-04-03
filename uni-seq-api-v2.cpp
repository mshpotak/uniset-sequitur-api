#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include <string>
#include <cstring>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <fstream>

#include <errno.h>


#define PORT_LOCALHOST "5678"
#define PORT_NETWORK   "5679"
#define UDP_BUFFER_LENGTH 1024
#define CLIENT 0
#define SERVER 1

using namespace std;

//variables
struct addrinfo *servinfo;
char buffer[UDP_BUFFER_LENGTH];
int sock_fd_localhost = -1;
int sock_fd_network = -1;

//functions
int set_addrinfo(const char ip[],
                 const char port[],
                 int family = AF_INET,
                 int socktype = SOCK_DGRAM,
                 int flags = AI_PASSIVE){

  struct addrinfo hints;
  int rv;

  memset(&hints, 0, sizeof(hints));

  hints.ai_family = family;
  hints.ai_socktype = socktype;
  hints.ai_flags = flags;

  if((rv = getaddrinfo(ip, port, &hints, &servinfo)) != 0){
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return -1;
  }

  return 0;
}

int set_socket(int type = CLIENT){
  int sock_fd;
  int yes = 1;
  if(type == CLIENT){
    if((sock_fd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol)) == -1){
  		perror("socket: error");
  		return -1;
  	}
    if(connect(sock_fd, servinfo->ai_addr, servinfo->ai_addrlen) == -1){
  		perror("connect: error");
  		return -1;
  	}
  }
  if(type == SERVER){
    if((sock_fd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol)) == -1){
      perror("socket: error");
  		return -1;
  	}

    if(setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {

      perror("setsockopt");
      exit(1);
    }

    if(bind(sock_fd, servinfo->ai_addr, servinfo->ai_addrlen) == -1) {
        close(sock_fd);
        perror("bind: error");
        return -1;
    }

    if(connect(sock_fd, servinfo->ai_addr, servinfo->ai_addrlen) == -1){
  		perror("connect: error");
  		return -1;
  	}
  }

  return sock_fd;
}



int main() {
  if(set_addrinfo("127.0.0.1", PORT_LOCALHOST) == -1){
    return 0;
  }
  if((sock_fd_localhost = set_socket(CLIENT)) == -1){
		return 0;
	}
  freeaddrinfo(servinfo);
  
  if(recv(sock_fd_localhost, buffer, UDP_BUFFER_LENGTH-1, 0) == -1){
    perror("send: error");
    return -1;
  }
    return 0;
}
