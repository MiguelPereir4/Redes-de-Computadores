/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define START 0 
#define FLAG_RCV 1
#define A_RCV 2
#define C_RCV 3
#define BCC_OK 4
#define STOP_ 5

#define F 0x5c
#define A 0x03
#define C 0x06
#define C_11 0x11
#define C_01 0x01
#define BCC A^C
#define BCC_11 A^C_11
#define BCC_01 A^C_01

volatile int STOP=FALSE;


void mens(int fd)
{
	unsigned char bufm[9];
	int res1, pos=1;
	
	
	bufm[0] = 0x5c; //FLAG
	bufm[1] = 0x03; //AD
	if(pos%2!=0) bufm[2] = 0x80; //C_Dados
	if(pos%2==0) bufm[2] = 0xc0;
	pos++;
	
	bufm[3] = bufm[1]^bufm[2]; //BCC1
	bufm[4] = 0x11; //Dados
	bufm[5] = 0x00;
	bufm[6] = 0x69;
	bufm[7] = bufm[4]^bufm[5]^bufm[6]; //BCC2
	bufm[8] = 0x5c; //F

	bufm[9] = '\n';

    res1 = write(fd,bufm,9);
    printf("%d bytes written\n", res1);
    
    printf("Vai fazer a rece\n");
    fflush(stdout);
    
    rec_RR(fd);

	
}

void rec_RR(int fd){
	
	printf("Abriu a rece \n");
	
	unsigned char buf[255]; int estado = 0;
	
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
            if(buf[0] == C_11){
                estado = C_RCV;
            }
            else if(buf[0]== C_01){
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
            if(buf[0] == BCC_11 || buf[0] == BCC_01){
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
            else if(buf[0]==F){
                estado = FLAG_RCV;
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
	printf("Recebida a Confirmação\n");

}




int main(int argc, char** argv)
{
    int fd,c, res, estado =0;
    struct termios oldtio,newtio;
    unsigned char bufw[4], buf[255];
    int i, sum = 0, speed = 0;

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

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
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


	bufw[0] = 0x5c; //FLAG
	bufw[1] = 0x03; //AD
	bufw[2] = 0x08; //C
	bufw[3] = buf[1]^buf[2]; //BCC
	bufw[4] = 0x5c; //F
  
  

    /*testing*/
    bufw[5] = '\n';

    res = write(fd,bufw,5);
    printf("%d bytes written\n", res);


    /*
    O ciclo FOR e as instruções seguintes devem ser alterados de modo a respeitar
    o indicado no guião
    */
	
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
            else if(buf[0]==F){
                estado = FLAG_RCV;
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
	
	
	mens(fd);
	
    sleep(1);
    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }


    close(fd);
    return 0;
}
