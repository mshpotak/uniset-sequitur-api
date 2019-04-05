#include <iostream>
#include <stdlib.h>
#include <string>
#include <cstring>

using namespace std;

#define CLIENT_PING 0


class sqtr_request{
    private:
        int code;
        string parameters;
        int parameters_amount;

        int check_code(int user_code){
            if(user_code == CLIENT_PING){
                code = user_code;
                return 0;
            } else {
                cout << "Invalid request code. Try *.help." << endl;
                return -1;
            }
        }
        int check_parameters(string user_parameters){
            if(code == CLIENT_PING){
                if(user_parameters.compare("0") == 0){
                    return 0;
                } else {
                    cout << "Invalid parameters. Try *.help." << endl;
                    return -1;
                }
            } else return -1;
        }

    public:
        //guided request: sequential argument input
        string guided(int user_code){
            if(check_code(user_code) == -1) return "\0";
            string user_parameters;
            cout << "Enter parameters: ";
            getline(cin, user_parameters);
            if(check_parameters(user_parameters) == -1) return "\0";

            parameters = user_parameters;
            return "Success!";
        }
        //advenced request: full argument input
        string advanced(int user_code, string user_parameters){
            if(check_code(user_code) == -1) return "\0";
            if(check_parameters(user_parameters) == -1) return "\0";

            parameters = user_parameters;
            return "Success!";
        }
        //help request: ask for request list
        //void help
};

int main()
{
    sqtr_request ping1,ping2;
    int code;
    string parameters;
    while(1)
    {
        cout << "Enter the request code: ";
        cin >> code;
        cin.ignore();
        cout << "Enter the request parameters: ";
        getline(cin, parameters);
        cout << endl;
        cout << "Code: " << code << "\tParameters:" << parameters << endl;

        scout << "Advanced function test: " << endl;
        cout << ping1.advanced(code, parameters) << endl;
        cout << "Guided function test: " << endl;
        cout << ping2.guided(code) << "\n\n";
    }
    return 0;
}
