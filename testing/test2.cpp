#include <iostream>
#include <stdlib.h>
#include <string>
#include <cstring>

using namespace std;



int main(int argc, const char* argv[])
{
    char str[] = "{-1 59 2 148235353571.1 15.88 73.39 1.04 0.34 0.10 0.25 148235353571.1 0.028 0.013 0.976 5.329 -1.199 -0.604 -0.200 -0.119 -0.440 1020.78 3 }";
    char * pch;
    printf ("Splitting string \"%s\" into tokens:\n",str);
    pch = strtok (str," {}");
    while (pch != NULL)
    {
    printf ("%s\n",pch);
    pch = strtok (NULL, " {}");
    }
    return 0;
}
