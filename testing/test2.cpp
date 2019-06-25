#include <iostream>
#include <stdlib.h>
#include <string>
#include <cstring>

using namespace std;

//char buffer[1024];

int main(int argc, const char* argv[])
{
    char str[] = "{-1 59 2 148235353571.1 15.88 73.39 1.04 0.34 0.10 0.25 148235353571.1 0.028 0.013 0.976 5.329 -1.199 -0.604 -0.200 -0.119 -0.440 1020.78 3 }";
    sscanf(str,"{%*d %*d %*d %[^_}]", str);
    string buff = str;
    cout << buff << endl;

    return 0;
}
