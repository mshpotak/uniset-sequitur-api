#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <poll.h>

#include <string>
#include <cstring>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <fstream>

#include <errno.h>

#include "seqreqclass.hpp"


#define PORT_SEQUITUR "5678"
#define UDP_BUFFER_LENGTH 256
#define UDP_BUFFER_SIZE 255

using namespace std;

//variables
char buffer[UDP_BUFFER_LENGTH];
int sock_fd = -1;
int hashcode = -1;

//functions
int set_connection(const char ip[], const char port[], int family = AF_INET, int socktype = SOCK_DGRAM, int flags = AI_PASSIVE){

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

int send_request(Seq_Request x){
	//form a request
	hashcode = rand() % 9000 + 1000;
    sprintf(buffer, "{%d %d %s}", hashcode, x.get_code(), x.get_parameters());

	//send() a request to Sequitur
	if(send(sock_fd, buffer, UDP_BUFFER_SIZE, 0) == -1){
		perror("send error: ");
		return 1;
	}

	cout << "Request sent...\n" << buffer << endl;
	return 0;
}

int recv_response(int timeout_ms){

    //initialize poll() variables
    int poll_res = 0;
    struct pollfd sfd;
    sfd.fd = sock_fd;
    sfd.events = POLLIN;

    //wait until data is available
    do{
        if((poll_res = poll(&sfd, 1, timeout_ms)) == -1){
            perror("poll error:");
            continue;
        };
        if(poll_res > 0){
            //check for error
            if(sfd.revents & POLLERR){
                perror("poll revents error:");
                continue;
            }
            //recv() if data is available
            if(sfd.revents & POLLIN){
                if(recv(sock_fd, buffer, UDP_BUFFER_SIZE, 0) == -1){
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

int ping_test(){
    Seq_Request req0(0);
    if(set_connection("192.168.2.105", PORT_SEQUITUR) != 0) return 0;
    if(send_request(req0) != 0) return 0;
    if(recv_response(10) != 0) return 0;
    return 0;
}

// void input_check(int argc, const char* argv[]){
//     string user_input;
//
//     //c-string
//     if(argc > 1){
//         if(argv[1][0] == '-'){
//             switch(argv[1][1]){
//                 case 'u':
//                     cout << "\n--USER MODE--\n" << endl;
//                     cout << "Input request code and its parameters: " << endl;
//                     getline(cin, user_input);
//                     cout << "\n-- INPUT COMPLETE --\n" << endl;
//                     break;
//                 case 'r':
//
//                     continue;
//
//                     break
//                 default
//
//                 break
//             }
//         }
//     }
//     cout << "\nUser input: " << endl;
//     cout << user_input << endl;
//     cout << "-- INPUT COMPLETE --\n" << endl;
// }

//main
int main(int argc, const char* argv[]) {
    Seq_Request ForwardDataEnable(3, "positionforwardenabled 1 1 0");

    if(set_connection("192.168.2.105", PORT_SEQUITUR) != 0) return 0;
    if(send_request(ForwardDataEnable) != 0) return 0;
    if(recv_response(10) != 0) goto ending;


    int file_fd;
    if((file_fd = open("Output.csv", (O_RDWR|O_CREAT|O_TRUNC), 0644)) == -1) goto ending;
    char * data;

    data = strtok(buffer, " {}");
    while(data != NULL){
        if(write(file_fd,(const char*)data, strlen(data))) == -1) break;
        data = strtok (NULL, " {}");
    }

    ending:
    Seq_Request ForwardDataDisable(3, "positionforwardenabled 0 1 0");
    if(send_request(ForwardDataDisable) != 0) return 0;
    return 0;
}
