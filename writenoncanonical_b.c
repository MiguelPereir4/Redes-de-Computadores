/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
//#include <linklayer.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1
#define MAX_PAYLOAD_SIZE 1000


#define START 0 
#define FLAG_RCV 1
#define A_RCV 2
#define C_RCV 3
#define BCC_OK 4
#define STOP_ 5

#define F 0x5c
#define A 0x03
#define C_SET 0x07
#define C_UA 0x06
#define C_11 0x11
#define C_01 0x01
#define C_DISC 0x0A
#define BCC_UA A^C_UA
#define BCC_SET A^C_SET
#define BCC_11 A^C_11
#define BCC_01 A^C_01
#define BCC_DISC A^C_DISC

#define C_05 0x05   //C de rejeição
#define C_15 0x15

#define ESCAPE 0x5d
#define ESCAPE_FLAG 0x7c    
#define ESCAPE_ESCAPE 0x7d

typedef struct linkLayer{
    char serialPort[50];
    int role; //defines the role of the program: 0==Transmitter, 1=Receiver
    int baudRate;
    int numTries;
    int timeOut;
} linkLayer;

volatile int STOP=FALSE;
unsigned char bufw[6], buf[255];
int fd;

void UA_W()
{
    int res;
    
    bufw[0] = 0x5c; //FLAG
	bufw[1] = 0x03; //A
	bufw[2] = 0x06; //C_UA
	bufw[3] = 0x05; //BCC
	bufw[4] = 0x5c; //F
    bufw[5] = '\n';

    res = write(fd,bufw,7);
    printf("%d bytes written\n", res);
}

void connect()
{
    printf("Entrou connect\n");
    
    int res;

    printf("Vai escrever no buffer\n");
    
    bufw[0] = 0x5c; //FLAG
	bufw[1] = 0x03; //A
	bufw[2] = 0x07; //C
	bufw[3] = bufw[1]^bufw[2]; //BCC
	bufw[4] = 0x5c; //F
    bufw[5] = '\n';

    printf("Vai escrever o write\n");

    res = write(fd,bufw,7);
    printf("%d bytes written\n", res);
}

void disconnect_w()
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
            for(x=length; x>i; x--)
            {
                aux[x] = aux[x-1];
            }
        } else if (input[i] == ESCAPE) {
            output[j++] = ESCAPE;
            output[j++] = ESCAPE_ESCAPE;
            for(x=length; x>i; x--)
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

void mens(unsigned char buffer[], int bytes_read)
{
	unsigned char bufm[MAX_PAYLOAD_SIZE], aux[2*MAX_PAYLOAD_SIZE];
    unsigned char stuffed_bufm[18];
    unsigned char BCC2_calc;
	int res1, pos, c=4, b=0,a=0,stuffed_length, controlo=0;
	

    for(b=0;b<=MAX_PAYLOAD_SIZE;b++)
    {
        bufm[b] = buffer[a];
        a++;
    }
    a=0;

    /*for(b=5;b <= MAX_PAYLOAD_SIZE; b++)
    {
        if(b==5) {
            BCC2_calc = bufm[b-1]^bufm[b];
        }
        else {BCC2_calc = bufm[b] ^ BCC2_calc;}
    }

    if(BCC2_calc != bufm[MAX_PAYLOAD_SIZE-3]){
    printf("Valor calculado de BCC2 não corresponde ao obtido.\n");
    printf("BCC2 Calculado: %X\nBCC2 Obtido: %X",BCC2_calc,bufm[MAX_PAYLOAD_SIZE-3]);
    return -1;
    }*/


    strcpy(aux, bufm);


    byte_stuffing(bufm, MAX_PAYLOAD_SIZE, stuffed_bufm, &stuffed_length, aux);


    for(b=4; b<=MAX_PAYLOAD_SIZE; b++)
    {
        if(a<=stuffed_length-1)
        {
            aux[b] = stuffed_bufm[a];
            a++;
        }
    }


    res1 = write(fd, aux, MAX_PAYLOAD_SIZE);
    printf("%d bytes written\n", res1);
    
    fflush(stdout);
    
    controlo = rec_RR();

    if(controlo != 0)
    {
        mens(bufm, bytes_read);
    }
    else return stuffed_length-1;

}

int rec_RR(){
		
	unsigned char buf[255]; int estado = 0;
    int controlo=0;
	
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
            else if(buf[0] == C_15 || buf[0] == C_05){
                controlo=1;
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
            else if(controlo == 1){
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

	if(controlo != 0)
    {
        controlo=0;
        return 1;
    }
    else return 0;

}

int llopen_W(struct linkLayer ll)
{

    printf("New termios structure set WRITE\n");

    int estado =0;
    struct termios oldtio,newtio;

    fd = open(ll.serialPort, O_RDWR | O_NOCTTY );
    if(fd<0){printf("Error\n");return-1;}

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

    printf("Vai entrar no connect\n");
    connect(fd);

    printf("Vai entrar no while\n");

    while(estado != STOP_){
        printf("Entrou no while\n");
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
            if(buf[0] == C_UA){
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
            if(buf[0] == BCC_UA){
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
            printf("\nConexão estabelecida.\n");
            break;
        default:
            printf("Default ativado, algo está incorreto.\n");
            break;
        }
    }

    if(estado == STOP_)
    {
        estado = 0;
        return 1;
    }

    estado = 0;
    return -1;

}

int llwrite(unsigned char buffer,int bytes_read)
{
    mens(buffer,bytes_read);
    
    return 1;
}

int llclose_W(struct linkLayer ll, int x)
{
    disconnect_w();

    int estado=0;

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
            if(buf[0] == C_DISC){
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
            if(buf[0] == BCC_DISC){
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
            close(fd);
            return -1;
            break;
        }

    }

    UA_W();


    printf("\n----DISCONECTADO----\n");

    close(fd);

    return 1;

    
}

/*int main(int argc, char** argv)
{
    int fd,c, res, estado =0;
    struct termios oldtio,newtio;
    int i, sum = 0, speed = 0;

   
    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");


	connect();


   
	
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
}*/
