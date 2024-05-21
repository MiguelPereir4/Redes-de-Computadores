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
#define C 0x07
#define BCC A^C
#define C_DISC 0x0A
#define BCC_DISC A^C_DISC

#define C_8 0x80
#define C_C 0xc0
#define BCC1_8 A^C_8
#define BCC1_C A^C_C
#define C_RR_S0 0x11
#define C_RR_S1 0x01

#define ESCAPE 0x5d
#define ESCAPE_FLAG 0x7c
#define ESCAPE_ESCAPE 0x7d

unsigned char bufaux[255];
unsigned char buf[255], bufw[4];

void UA(int fd)
{
    int res;
    
    bufw[0] = 0x5c; //FLAG
	bufw[1] = 0x03; //A
	bufw[2] = 0x06; //C_UA
	bufw[3] = 0x04; //BCC
	bufw[4] = 0x5c; //F
  

    /*testing*/
    bufw[5] = '\n';

    res = write(fd,bufw,5);
    printf("%d bytes written\n", res);
}

//ChatGPT
void byte_destuffing(const unsigned char *input, int length, unsigned char *output, int *destuffed_length) {
    int i, j = 0;
    printf("Inicio destuffing\n");
    for (i = 4; i < length; i++) {
        if (input[i] == ESCAPE) {
            if (input[i + 1] == ESCAPE_FLAG) {
                output[j++] = F;
            } else if (input[i + 1] == ESCAPE_ESCAPE) {
                output[j++] = ESCAPE;
            }
            i++; // Pular o próximo byte porque já foi tratado
        } else {
            output[j++] = input[i];
        }
    }
    *destuffed_length = j;
    printf("Fim destuffing\n");
}


void dados(int fd)
{


    int estado = 0, i=0, jaux=5, pos, a=0, b=0;
    unsigned char bufm[255], aux[255], BCC2, BCC2_r;

    printf("Vai entrar no while\n");


    while(estado != STOP_){
        read(fd, bufm, 1);
        bufaux[i] = bufm[0];
        i++;
        printf("%X \n", bufm[0]);
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
            if((bufm[0] == C_8) || (bufm[0] == C_C) ){
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
            printf("Entrou no DATA\n");
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
    BCC2_r = bufaux[i-1];

    strcpy(aux, bufaux);

    for(pos = 0; pos < 255; pos++)
    {
        printf("%X ", aux[pos]);
    }

    printf("\n\n");

    // Destuff the data
    unsigned char destuffed_buf[255];
    int destuffed_length;
    byte_destuffing(bufaux, i, destuffed_buf, &destuffed_length);

    for(pos = 0; pos < destuffed_length; pos++)
    {
        printf("%X ", destuffed_buf[pos]);
    }

    printf("\n\n");

    for(b=4; b<=255; b++)
    {
        if(a<=destuffed_length-1)
        {
            aux[b] = destuffed_buf[a];
            a++;
        }
    }

    for(b=4+destuffed_length; b<255; b++)
    {
        aux[b]=0;
    }

    aux[4+destuffed_length] = aux[0];
    
    for(pos = 0; pos < 255; pos++)
    {
        printf("%X ", aux[pos]);
    }

    for(jaux=5;jaux < (3+destuffed_length); jaux++)
    {
        if(jaux==5) {
            BCC2 = aux[jaux-1]^aux[jaux];
        }
        else {BCC2 = aux[jaux] ^ BCC2;}
    }
    printf("\nBCC CAlCUlADO: %X \n", BCC2);

    if(BCC2 == BCC2_r)
    {
        confirmacao(fd);
    }
    else reject(fd);
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

void reject(int fd)
{
    unsigned char bufrej[6]; int res;

    bufrej[0] = bufaux[0];
    bufrej[1] = bufaux[1];

    if(bufaux[2] == C_8) bufrej[2] = 0x15;
    if(bufaux[2] == C_C) bufrej[2] = 0x05;

    bufrej[3] = bufrej[1] ^bufrej[2];
    bufrej[4] = bufrej[0];
    bufrej[5]="\n";

    res = write(fd, bufrej,5);
    printf("\nFoi dada a rejeição\n");

}


int main(int argc, char** argv)
{
    int fd,c, res, estado=0;
    struct termios oldtio,newtio;

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

    UA(fd);

    dados(fd);

    printf("Dados foram recebidos e conf enviada\n");
    
    tcflush(fd, TCIOFLUSH);

    estado = 0;

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
            break;
        }

    }

    printf("\n----DISCONECTADO----\n");

    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */
    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    printf("Programa fechado.\n");
    return 0;
}
