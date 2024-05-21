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

#define ESCAPE 0x5d
#define ESCAPE_FLAG 0x7c
#define ESCAPE_ESCAPE 0x7d

volatile int STOP=FALSE;
unsigned char bufw[4], buf[255];

void connect(int fd)
{
    int res;
    
    bufw[0] = 0x5c; //FLAG
	bufw[1] = 0x03; //A
	bufw[2] = 0x07; //C
	bufw[3] = buf[1]^buf[2]; //BCC
	bufw[4] = 0x5c; //F
    bufw[5] = '\n';

    res = write(fd,bufw,5);
    printf("%d bytes written\n", res);
}

void disconnect(int fd)
{
    int res;
    
    bufw[0] = 0x5c; //FLAG
	bufw[1] = 0x03; //A
	bufw[2] = 0x0A; //C
	bufw[3] = buf[1]^buf[2]; //BCC
	bufw[4] = 0x5c; //F
    bufw[5] = '\n';

    res = write(fd,bufw,5);
    printf("%d bytes written\n", res);
}

//ChatGPT
void byte_stuffing(const unsigned char *input, int length, unsigned char *output, int *stuffed_length, unsigned char *aux) {
    int i, j = 0, x;

    printf("Inicio Stuffing\n");
    for (i = 4; i < length - 1; i++) {
        if (input[i] == F) {
            output[j++] = ESCAPE;
            output[j++] = ESCAPE_FLAG;
            for(x=255; x>i; x--)
            {
                aux[x] = aux[x-1];
            }
        } else if (input[i] == ESCAPE) {
            output[j++] = ESCAPE;
            output[j++] = ESCAPE_ESCAPE;
            for(x=255; x>i; x--)
            {
                aux[x] = aux[x-1];
            }
        } else {
            output[j++] = input[i];
        }
    }
    *stuffed_length = j;

    printf("Fim Stuffing\n");
}

void mens(int fd)
{
	unsigned char bufm[255], aux[255];
    unsigned char stuffed_bufm[18];
	int res1, pos, c=4, b=0,a=0,stuffed_length;
	
	
	bufm[0] = 0x5c; //FLAG
	bufm[1] = 0x03; //AD
	if(b%2!=0) bufm[2] = 0x80; //C_Dados
	if(b%2==0) bufm[2] = 0xc0;
	b++;
	
	bufm[3] = bufm[1]^bufm[2]; //BCC1
	bufm[4] = 0x11; //Dados
	bufm[5] = 0x5c;
    bufm[6] = 0x12;
    bufm[7] = 0x13;
    bufm[8] = 0x5d;
    bufm[9] = 0x14;
	bufm[10] = 0x69;
    for(b=5;b <= 13-3; b++)
    {
        if(b==5) {
            bufm[11] = bufm[b-1]^bufm[b];
        }
        else {bufm[11] = bufm[b] ^ bufm[11];}
    }
	bufm[12] = 0x5c; //F
	bufm[13] = '\n';

    printf("----BCC---- %X\n", bufm[11]);

    strcpy(aux, bufm);

    for(pos = 0; pos < 255; pos++)
    {
        printf("%X ", aux[pos]);
    }

    printf("\n\n");

    byte_stuffing(bufm, 13, stuffed_bufm, &stuffed_length, aux);

    for(pos = 0; pos < 255; pos++)
    {
        printf("%X ", aux[pos]);
    }

    printf("\n\n");

    for(b=4; b<=255; b++)
    {
        if(a<=stuffed_length-1)
        {
            aux[b] = stuffed_bufm[a];
            a++;
        }
    }
    
    for(pos = 0; pos < 255; pos++)
    {
        printf("%X ", aux[pos]);
    }

    printf("\n");


    res1 = write(fd, aux, 255);
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


	connect(fd);


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

    disconnect(fd);
	
    sleep(1);
    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }


    close(fd);
    return 0;
}
