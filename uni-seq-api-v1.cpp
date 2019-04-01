#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <errno.h>
#include <chrono>
#include <poll.h>

using namespace std;

#define PORT "5678"
#define UDP_BUFFER_LENGTH 1024

//API codes
#define CLIENT_GET_PARAMETER 4
#define CLIENT_SET_ANCHOR_INFO 11
#define CLIENT_GET_TAG_POSITION 14
#define CLIENT_GET_RANGE 50
#define CLIENT_GET_SENSORS 51
#define CLIENT_SCAN 56

//buffer for incoming UDP packets
char buffer[UDP_BUFFER_LENGTH];

//variables for getaddrinfo()
struct addrinfo *servinfo;

int sockfd;
//structures
struct recvData{
	int info[2];
	double data[11];
};

int sent_req = 0;
int recv_req = 0;


//functions
int writeToFile(string a, int b, int c);
int setAddrInfo(char const IP[]);
int setSocketConnection();
int sendRequest(short RequestCode = 0, char const Parameters[] = {}, int sizeParam = 0);
int recvResponse(int timeout_u, int hashcode);
int recordTagMovement(int state = 0, int timer = 60, double frequency = 0);
int getParameter(char const parameters[], int sizeParam);
int setAnchorPositions();
int removeString(char *str, char const *string_to_remove);
recvData formatData(int CLIENT_REQUEST_NUMBER);

int main(int argc, char const *argv[]){

	if(argc == 1) {
		printf("\n<< Not enough arguments >>\n");
		printf("\nRequired arguments:\n1. Tag IP Adress\nSecondary arguments:\n2. Recoring time (seconds)\n3. Recording frequency (Hz)\n\n");
		return 0;

	}

	//setting up the socket()
	if(setAddrInfo(argv[1]) == -1){
		return 0;
	}
	if((sockfd = setSocketConnection()) == -1){
		return 0;
	};


	setAnchorPositions();

	remove("Loops-n-hoops.csv");
	//record tag positions from program input
	if(argc == 2){

		recordTagMovement(1);

	} else if(argc == 3){

		recordTagMovement(1, strtol(argv[2],NULL,10));

	} else if(argc == 4){

		recordTagMovement(1, strtol(argv[2],NULL,10), strtod(argv[3],NULL));

	} else if(argc > 5){
		printf("\n<< Too much arguments >>\n");
		printf("\nRequired arguments:\n1. Tag IP Adress\nSecondary arguments:\n2. Recoring time (seconds)\n3. Recording frequency (Hz)\n\n");
		return 0;
	}

	//getParameter

	//	char parameter[] = "ipaddress";
	//	getParameter(parameter,sizeof(parameter));

	//close() the socket
	close(sockfd);
	freeaddrinfo(servinfo);
	cout << "\n\nSent requests: " << sent_req << "\nReceived requests: " << recv_req << endl;
	return 0;
}

int sendRequest(short RequestCode, char const Parameters[], int sizeParam){

	//form a request
	int Hashcode = rand() % 9000 + 1000;
	cout << "Hashcode assigned: " << Hashcode << endl;
	cout << "Size of parameters: " << sizeParam << " bytes" << endl;
	char command[1+4+1+2+1+sizeParam+1];

	sprintf(command, "{%d %d %s}", Hashcode, RequestCode, Parameters);
	cout << "Hashcode formatted: " << Hashcode << endl;
	//send() a request to Sequitur
	if(send(sockfd, command, strlen(command), 0) == -1){
		perror("send: error");
		return -1;
	}

	printf("Request sent...\n");
	printf("%s\n\n", command);

	cout << "Hashcode on return: " << Hashcode << "\n\n";

	sent_req++;
	return Hashcode;
}

int recvResponse(int timeout_u, int hashcode){
	cout << "Sent hashcode: " << hashcode << endl;

	//initiating ppoll() variables
	int poll_res;
	struct pollfd sfd;
	sfd.fd = sockfd;
	sfd.events = POLLIN;

	//set recieving timeout

	/*
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = timeout_u;

	if(setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1){
		perror("setsockopt: error");
		return -1;
	}
	*/

	//hashcode check loop
	int recv_hashcode;
	auto start = chrono::high_resolution_clock::now();
	auto finish = chrono::high_resolution_clock::now();

	int i = 0, j = 0;
	do{
		do{
			poll_res = poll(&sfd,1,timeout_u/1000);
			cout << "Event bit mask: " << poll_res << endl;
			if(poll_res > 0){
				cout << "Poll positive... POLLIN is " << (sfd.revents & POLLIN) <<  endl;
				if(sfd.revents & POLLIN){
				//recv() an answer from Sequitur
					if(recv(sockfd, buffer, UDP_BUFFER_LENGTH-1, 0) == -1){
						perror("receive: error");
						return -1;
					}
					printf("Received message: %s\n", buffer);
				}
				if(sfd.revents & POLLERR){
					perror("poll: error");
					continue;
				}
			}
			cout << "Poll loop condition is " << !(sfd.revents & POLLIN) << endl;
			i++;
		}while(!(sfd.revents & POLLIN));

		//initiate hashcode timeout
		finish = chrono::high_resolution_clock::now();
		cout << "Receive latency (ms): " << double(chrono::duration_cast<chrono::microseconds>(finish-start).count())/1000 << endl;

		//read received hashcode
		sscanf(buffer,"{%d", &recv_hashcode);
		cout << "\nSent hashcode:\t" << hashcode << "\nReceived hashcode:\t" << recv_hashcode << "\n\n";
		j++;
		//if(chrono::duration_cast<chrono::microseconds>(finish-start).count() > (10*timeout.tv_usec)) return -1;
	} while(recv_hashcode != hashcode);
  cout << "Poll loops: " << i << "\tHashcode loops: " << j << endl;
	writeToFile("Loops-n-hoops.csv", i, j);
	printf("%s\n\n", buffer); //print recived message
	//getchar();
	recv_req++;
	return chrono::duration_cast<chrono::microseconds>(finish-start).count();
}

int setAddrInfo(char const IP[]){

	//setting hints structure for getaddrinfo()
	struct addrinfo hints;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE;

	//getaddrinfo() call and error check

	if(getaddrinfo(IP, PORT, &hints, &servinfo) != 0){
		perror("getaddrinfo: error");
		return -1;
	}

	return 0;
}

int setSocketConnection(){

	//initializing socket()
	if((sockfd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol)) == -1){
		perror("socket: error");
		return -1;
	}

	//initializing connect()
	if(connect(sockfd, servinfo->ai_addr, servinfo->ai_addrlen) == -1){
		perror("connect: error");
		return -1;
	}

	return sockfd;
}

int removeString(char *str, char const *string_to_remove){
	int found = 0;
	char *p, *p1;
	p = strstr(str, string_to_remove);
	long l;
	p1 = p + strlen(string_to_remove);
	l = strlen(p1);
	if (p != NULL) { found = 1; memcpy(p, p1, l + 1); }
	return found;
}

recvData formatData(int CLIENT_REQUEST_NUMBER){
	recvData rslt;

	if(CLIENT_REQUEST_NUMBER == CLIENT_GET_TAG_POSITION){
		sscanf(buffer, "{%d %d %lf %lf %lf %lf %lf %lf %lf}",
						 &rslt.info[0], &rslt.info[1], &rslt.data[0], &rslt.data[1], &rslt.data[2], &rslt.data[3], &rslt.data[4], &rslt.data[5], &rslt.data[6]);

		return rslt;
	}

	if(CLIENT_REQUEST_NUMBER == CLIENT_GET_SENSORS){
		sscanf(buffer, "{%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf}",
						 &rslt.info[0], &rslt.info[1], &rslt.data[0], &rslt.data[1], &rslt.data[2], &rslt.data[3], &rslt.data[4], &rslt.data[5], &rslt.data[6], &rslt.data[7], &rslt.data[8], &rslt.data[9], &rslt.data[10]);

		return rslt;
	}

	if(CLIENT_REQUEST_NUMBER == CLIENT_SET_ANCHOR_INFO){
		int info;
		static double data;
		sscanf(buffer, "{%d %d}",
						 &rslt.info[0], &rslt.info[1]);

		return rslt;
	}
}

int recordTagMovement(int state, int timer, double frequency){

	//initialize record parameters

	if(state == 0){
		cout << "Enter timer: ";
		cin >> timer;
		cout << "Enter frequency: ";
		cin >> frequency;
		cout << "\033c" << endl;
	}
	//initialize request parameters
	char parameters[] = "0";
	int sizeParam = strlen(parameters);
	int hashcode;
	int timeout_u = 10*1000;
	recvData resp;

	//initialize file output
	ofstream file;
	time_t curtime;
	time(&curtime);
	char *filename = ctime(&curtime);
	strcat(filename,".csv");
	file.open(filename);

	//record data
	int time0 = time(&curtime);
	int prcss_time;

	//maximum frequency
	if(frequency == 0){
		while(timer > difftime(time(&curtime),time0)){

			//send and receive the UDP packet
			if((hashcode = sendRequest(CLIENT_GET_TAG_POSITION, parameters, sizeParam)) == -1) continue;
			if((prcss_time = recvResponse(timeout_u, hashcode)) == -1) continue;

			//format the data and write it to the file
			resp = formatData(CLIENT_GET_TAG_POSITION);
			file << setprecision(13) << resp.data[0] << ";";
			file << resp.data[1] << ";" << resp.data[2] << ";" << resp.data[3] << ";" << resp.data[4] << ";" << resp.data[5] << ";" << resp.data[6] << ";" << prcss_time << endl;
		}
	//manual frequency (Hz)
	} else {
		//set frequency wait time and correction time
		int t_freq = int((1.0/frequency)*1000000);
		int t_fix = 0;

		auto start = chrono::high_resolution_clock::now();
		auto finish = chrono::high_resolution_clock::now();

		while(timer > difftime(time(&curtime),time0)){
			printf("\033c"); //clean console
			//start frequency correction timer
			start = chrono::high_resolution_clock::now();

			//send and receive the UDP packet
			if((hashcode = sendRequest(CLIENT_GET_TAG_POSITION, parameters, sizeParam)) == -1) continue;
			if((prcss_time = recvResponse(timeout_u, hashcode)) == -1) continue;

			//format the data and write it to the file
			resp = formatData(CLIENT_GET_TAG_POSITION);
			file << setprecision(13) << resp.data[0] << ";";
			file << resp.data[1] << ";" << resp.data[2] << ";" << resp.data[3] << ";" << resp.data[4] << ";" << resp.data[5] << ";" << resp.data[6] << ";" << prcss_time << endl;

			//stop frequency correction timer
			finish = chrono::high_resolution_clock::now();
			t_fix = chrono::duration_cast<chrono::microseconds>(finish-start).count();
			if(t_fix >= t_freq) continue;

			//make correction to waiting time == correct frequency
			usleep(t_freq - t_fix);
			//cout << "t_freq: " << t_freq << "\tt_fix: " << t_fix << "\tu_sleep: " << t_freq - t_fix << endl;
		}
	}

	file.close();

	return 0;
}

int getParameter(char const parameters[], int sizeParam){
	int hashcode;

	hashcode = sendRequest(CLIENT_GET_PARAMETER, parameters, sizeParam);
	recvResponse(100000, hashcode);
	return 0;
}

int setAnchorPositions(){
	int hashcode;
	int errorcode;
	const char* parameters[] = {"1 1 10205F1310000DE1 -2.478457 2.6242 0.695852",
															"2 1 10205F1310001423 2.542345 2.645767 2.1514",
															"3 1 10205F1310001422 -2.469393 -1.541162 2.145",
															"4 1 10205F1310001425 2.550327 -1.541984 0.705067"};
  cout << "\n\n-------------------------" << endl;
	for(int i = 0; i < 4; i++){
		hashcode = sendRequest(CLIENT_SET_ANCHOR_INFO, parameters[i], strlen(parameters[i]));
		do{
			cout << "Sent hashcode in setAnchorPositions: " << hashcode << endl;
			recvResponse(100*1000, hashcode);

			recvData resp = formatData(CLIENT_SET_ANCHOR_INFO);

			if(resp.info[1] == 255){
				perror("setanchorinfo: error");
			}
			errorcode = resp.info[1];

			cout << "-------------------------" << endl;
		}while(errorcode != 0);
	}
	printf("\n\nAnchor Positions are all set!\n\n");
	getchar();
	return 0;
}

int writeToFile(string a, int b, int c){
	ofstream file;
	file.open(a,ios_base::app);
	file << b << ";" << c << endl;
	file.close();
	return 0;
}
