/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define START 0 
#define FLAG_RCV 1
#define A_RCV 2
#define C_RCV 3
#define BCC_OK 4
#define BCC1_OK8 5
#define BCC1_OKC 6
#define DATA 7
#define STOP_ 8

#define F 0x5c
#define A 0x03
#define C 0x08
#define BCC A^C

#define C_8 0x80
#define C_C 0xc0
#define BCC1_8 A^C_8
#define BCC1_C A^C_C
#define C_RR_S0 0x11
#define C_RR_S1 0x01

unsigned char bufaux[255];

void dados(int fd)
{
    int estado = 0, i=0, jaux=4;
    unsigned char bufm[255],aux, BCC2;

    while(estado != STOP_){
        read(fd, bufm, 1);
        bufaux[i] = bufm[0];
        i++;
        printf("%X ", bufm[0]);
        switch (estado)
        {
        case START:
            if(bufm[0] == F){
                estado = FLAG_RCV;
            }
            else{
                estado = START;
            }
            break;
        case FLAG_RCV:
            if(bufm[0] == A){
                estado = A_RCV;
            }
            else if(bufm[0]==F){
                estado = FLAG_RCV;
            }
            else{
                estado = START;
            }
            break;
        case A_RCV:
            if(bufm[0] == C_8){
                estado = C_RCV;
            }
            else if(bufm[0]==F){
                estado = FLAG_RCV;
            }
            else{
                estado = START;
            }
            break;
        case C_RCV:
            if(bufm[0] == BCC1_8){
                estado = BCC1_OK8;
            }
            else if(bufm[0]==BCC1_C){
                estado = BCC1_OKC;
            }
            else if(bufm[0]==F){
                estado = FLAG_RCV;
            }
            else{
                estado = START;
            }
            break;
        case BCC1_OK8:
            if(bufm[0] == F){
                estado = STOP_;
            }
            else{
                estado = DATA;
            }
            break;
        case BCC1_OKC:
            if(bufm[0] == F){
                estado = STOP_;
            }
            else{
                estado = DATA;
            }
            break;
        case DATA:
            if(bufm[0] == F){
                estado = STOP_;
            }
            break;
        case STOP_:
            estado = STOP_;
            break;
        default:
            printf("Default ativado, algo está incorreto.\n");
            break;
        }
    } 

    i--;
    printf("FLAG RECEBIDA: %X \n", bufaux[i]);
    printf("BCC2 RECEBIDO: %X \n", bufaux[i-1]);
    aux = bufaux[i-1];

    while(jaux<i)
    {
        if(jaux==5) {
            BCC2 = bufaux[jaux-1]^bufaux[jaux];
        }
        else {BCC2 = bufaux[jaux-1] ^ BCC2;}
        jaux++;

    }

    printf("BCC CAlCUlADO: %X \n", BCC2);

    confirmacao(fd);
    printf("Sai da conf\n");


}

void confirmacao(int fd)
{
    unsigned char bufconf[6]; int res;

    bufconf[0] = bufaux[0];
    bufconf[1] = bufaux[1];

    if(bufaux[2] == C_8) bufconf[2] = 0x11;
    if(bufaux[2] == C_C) bufconf[2] = 0x01;

    bufconf[3] = bufconf[1] ^bufconf[2];
    bufconf[4] = bufconf[0];
    bufconf[5]="\n";

    res = write(fd, bufconf,5);
    printf("\nFoi dada a confirmação\n");

}



int main(int argc, char** argv)
{
    int fd,c, res, estado=0;
    struct termios oldtio,newtio;
    unsigned char buf[255], bufw[4];


    if ( (argc < 2) ||
    
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    fflush(stdout);


    while(estado != STOP_){
        read(fd, buf, 1);
        printf("%X ", buf[0]);
        switch (estado)
        {
        case START:
            if(buf[0] == F){
                estado = FLAG_RCV;
            }
            else{
                estado = START;
            }
            break;
        case FLAG_RCV:
            if(buf[0] == A){
                estado = A_RCV;
            }
            else if(buf[0]==F){
                estado = FLAG_RCV;
            }
            else{
                estado = START;
            }
            break;
        case A_RCV:
            if(buf[0] == C){
                estado = C_RCV;
            }
            else if(buf[0]==F){
                estado = FLAG_RCV;
            }
            else{
                estado = START;
            }
            break;
        case C_RCV:
            if(buf[0] == BCC){
                estado = BCC_OK;
            }
            else if(buf[0]==F){
                estado = FLAG_RCV;
            }
            else{
                estado = START;
            }
            break;
        case BCC_OK:
            if(buf[0] == F){
                estado = STOP_;
            }
            else{
                estado = START;
            }
            break;
        case STOP_:
            estado = STOP_;
            printf("STOP atingido.\n");
            break;
        default:
            printf("Default ativado, algo está incorreto.\n");
            break;
        }

    }
    
    printf("\n");
    printf("STOP atingido.\n");

    bufw[0] = 0x5c; //FLAG
	bufw[1] = 0x03; //AD
	bufw[2] = 0x06; //C_UA
	bufw[3] = 0x04; //BCC
	bufw[4] = 0x5c; //F
  

    /*testing*/
    bufw[5] = '\n';

    res = write(fd,bufw,5);
    printf("%d bytes written\n", res);



    dados(fd);

    printf("Dados foram recebidos e conf enviada\n");

    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */
    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    printf("Programa fechado.\n");
    return 0;
}
