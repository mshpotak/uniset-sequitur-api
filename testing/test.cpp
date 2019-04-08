#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <poll.h>
#include <unistd.h>


using namespace std;

int main()
{
    char buffer[128];
    int buffer_size = 128;
    char* new_buffer;
    int bytes_rd;
    int timeout_ms = 5000;
    int poll_res = 0;
    struct pollfd sfd;
    sfd.fd = STDIN_FILENO;
    sfd.events = POLLIN;

    while(1){
        memset(buffer, 0, buffer_size);
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
                bytes_rd = read(STDIN_FILENO, buffer, buffer_size);
                cout << "BYTES READ: " << bytes_rd << endl;
                if(bytes_rd == 1) new_buffer = new char[bytes_rd];
                else new_buffer = new char[bytes_rd-1];
                new_buffer = buffer;
                if(buffer[0] == '\n') printf("You pressed ENTER.\n");
                printf("\nTYPED: '%s' END\n", buffer);
                cout << "\nTYPED: '" << new_buffer << "' END" << endl;
            }
        }
    }

    return 0;
}
