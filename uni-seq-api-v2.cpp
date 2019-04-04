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


#define PORT_SEQUITUR "5678"
#define UDP_BUFFER_LENGTH 256
#define UDP_BUFFER_SIZE 255

using namespace std;

//variables
char buffer[UDP_BUFFER_LENGTH];
int sock_fd = -1;
int hashcode = -1;

//classes
struct request{
        int code;
        char parameters[];
        int parameters_size;
};

request req0;
    req0.code = 0;
    req0.parameeters = "0";
    req0.parameters_size = sizeof(req0.parameters);

//functions
int set_connection(const char ip[],
                 const char port[],
                 int family = AF_INET,
                 int socktype = SOCK_DGRAM,
                 int flags = AI_PASSIVE){

    struct addrinfo hints;
    struct addrinfo *servinfo;
    int return_value;

    memset(&hints, 0, sizeof(hints));

    hints.ai_family = family;
    hints.ai_socktype = socktype;
    hints.ai_flags = flags;

    if((return_value = getaddrinfo(ip, port, &hints, &servinfo)) != 0){
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(return_value));
        return 1;
    }

    if((sock_fd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol)) == -1){
        perror("socket error: ");
        return 1;
    }
    if(connect(sock_fd, servinfo->ai_addr, servinfo->ai_addrlen) == -1){
        perror("connect error: ");
        return 1;
    }
    freeaddrinfo(servinfo);
    return 0;
}

int send_request(int request_code, char parameters[], int size_parameters){

	//form a request
	hashcode = rand() % 9000 + 1000;
    sprintf(buffer, "{%d %d %s}", hashcode, request_code, parameters);

	//send() a request to Sequitur
	if(send(sockfd, buffer, UDP_BUFFER_SIZE, 0) == -1){
		perror("send error: ");
		return 1;
	}

	cout << "Request sent...\n" << command << endl;
	return 0;
}

int recv_response(int timeout_ms){

    //initialize poll() variables
    struct pollfd sfd;
    sfd.fd = sockfd;
    sfd.events = POLLIN;

    //wait until data is available
    do{
        poll_res = poll(&sfd,1, timeout_ms);
        if(poll_res > 0){
            //check for error
            if(sfd.revents & POLLERR){
                perror("poll error:");
                continue;
            }
            //recv() if data is available
            if(sfd.revents & POLLIN){
                if(recv(sockfd, buffer, UDP_BUFFER_SIZE, 0) == -1){
                    perror("receive error:");
                    return -1;
                }
                cout << "Response received...\n" << buffer << "\n\n";
            }
        }
    } while(!(sfd.revents & POLLIN));
    return 0;
}

int check_hashtag(){
    int recv_hashcode;
    sscanf(buffer,"{%d", &recv_hashcode);
    if(hashcode == recv_hashcode) return 1;
    return 0;
}


//main
int main() {
    if(set_connection("192.168.2.105", PORT_SEQUITUR)) return 0;
    if(send_request(req0.code, req0.parameters, req0.parameters_size)) return 0;
    if(recv_response(10)) return 0;
    return 0;
}
