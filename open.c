#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
//#include <linklayer.h>
//#include <writenoncanonical_a.o>
//#include <noncanonical.o>

typedef struct linkLayer{
    char serialPort[50];
    int role; //defines the role of the program: 0==Transmitter, 1=Receiver
    int baudRate;
    int numTries;
    int timeOut;
} linkLayer;



int llopen(struct linkLayer ll){

    printf("Entrou no llopen\n");
    
    if (ll.role == 0) {llopen_W(ll); printf("Exe o llopen_w\n");;return 1;}
    else {llopen_R(ll); printf("Exe o llopen_r\n");return 1;}
    return -1;
}

int llclose(struct linkLayer ll, int x)
{
    if(ll.role == 0){llclose_W(ll,x);return 1;}
    else {llclose_R(ll,x);return 1;}
    return -1;
}
