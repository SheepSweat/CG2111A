
// Routines to create a TLS client
#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "constants.h"

// to remove need for enter
//#include <conio.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

// Tells us that the network is running.
static volatile int networkActive=0;
static volatile clock_t last_time=0;
static volatile clock_t current_time;


void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printf("Command / Status OK\n");
			break;

		case RESP_BAD_PACKET:
			printf("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printf("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printf("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;

		default:
			printf("PI IS CONFUSED!\n");
	}
}

void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));

	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", data[0]);
	printf("Right Forward Ticks:\t\t%d\n", data[1]);
	printf("Left Reverse Ticks:\t\t%d\n", data[2]);
	printf("Right Reverse Ticks:\t\t%d\n", data[3]);
	printf("Left Forward Ticks Turns:\t%d\n", data[4]);
	printf("Right Forward Ticks Turns:\t%d\n", data[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", data[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", data[7]);
	printf("Forward Distance:\t\t%d\n", data[8]);
	printf("Reverse Distance:\t\t%d\n", data[9]);
	printf("\n---------------------------------------\n\n");
}

void handleMessage(const char *buffer)
{
	printf("MESSAGE FROM ALEX: %s\n", &buffer[1]);
}

void handleCommand(const char *buffer)
{
	// We don't do anything because we issue commands
	// but we don't get them. Put this here
	// for future expansion
}

void handleNetwork(const char *buffer, int len)
{
	// The first byte is the packet type
	int type = buffer[0];

	switch(type)
	{
		case NET_ERROR_PACKET:
		handleError(buffer);
		break;

		case NET_STATUS_PACKET:
		handleStatus(buffer);
		break;

		case NET_MESSAGE_PACKET:
		handleMessage(buffer);
		break;

		case NET_COMMAND_PACKET:
		handleCommand(buffer);
		break;
	}
}

void sendData(void *conn, const char *buffer, int len)
{
	int c;
	printf("\nSENDING %d BYTES DATA\n\n", len);
	if(networkActive)
	{
		/* TODO: Insert SSL write here to write buffer to network */

		c = sslWrite(conn, buffer, len);
		/* END TODO */	
		networkActive = (c > 0);
	}
}

void *readerThread(void *conn)
{
	char buffer[128];
	int len;

	while(networkActive)
	{
		/* TODO: Insert SSL read here into buffer */

        printf("read %d bytes from server.\n", len);
		len = sslRead(conn, buffer, sizeof(buffer));
		/* END TODO */

		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");
    
    /* TODO: Stop the client loop and call EXIT_THREAD */
    stopClient();
    EXIT_THREAD(conn);

    /* END TODO */

    return NULL;
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

// void getParams(int32_t *params)
// {
// 	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
// 	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
// 	scanf("%d %d", &params[0], &params[1]);
// 	flushInput();
// }
// char getch() {
//     struct termios oldattr, newattr;
//     char ch;
//     tcgetattr(STDIN_FILENO, &oldattr);
//     newattr = oldattr;
//     newattr.c_lflag &= ~(ICANON | ECHO);
//     tcsetattr(STDIN_FILENO, TCSANOW, &newattr);
//     ch = getchar();
//     tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);
//     return ch;
// }
void enable_raw_mode() {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &newt);
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

void disable_raw_mode() {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &newt);
    newt.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

void set_nonblocking_mode() {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void *writerThread(void *conn)
{
	int quit=0;
    enable_raw_mode();
   set_nonblocking_mode();
    printf("Command (w=forward, s=reverse, a=turn left, d=turn right, f=setting1, g=setting2, h=setting3, z=ultrasonic, c=colour, x=RPi camera, q=Servo open, e=Servo close,  r=get stats, t=clear data, y=stop, o=open trap, p=shake v=exit)\n");

	while(!quit)
	{
		char ch;
		// ch = getch();
//		std::cout << "You typed: " << static_cast<char>(ch) << std::endl;
//	scanf("%c", &ch);
		 if (read(STDIN_FILENO, &ch, 1) > 0) {
           	 printf("You typed: %c\n", ch);
		 current_time =clock();
		double time_diff = (double)(current_time - last_time)/CLOCKS_PER_SEC;
		if (last_time ==0 || time_diff > 3){
		// Purge extraneous characters from input stream
		//flushInput();

		char buffer[10];
//		int32_t params[2];

		buffer[0] = NET_COMMAND_PACKET;
		switch(ch)
		{
			case 'w':
			case 'W':
			case 's':
			case 'S':
			case 'a':
			case 'A':
			case 'd':
			case 'D':
			case 'f':
			case 'F':
			case 'g':
			case 'G':
			case 'h':
			case 'H':
			case 'z':
			case 'Z':
			case 'c':
			case 'C':
			case 'x':
			case 'X':
			case 'q':
			case 'Q':
			case 'e':
			case 'E':
			case 'r':
			case 'R':
			case 't':
			case 'T':
			case 'v':
			case 'V':
			case 'y':
			case 'Y':
			case 'o':
			case 'O':
			case 'p':
			case 'P':
						buffer[1] = ch;
						sendData(conn,buffer,sizeof(buffer));
						break;
			// case 'f':
			// case 'F':
			// case 'b':
			// case 'B':
			// case 'l':
			// case 'L':
			// case 'r':
			// case 'R':
			// 			getParams(params);
			// 			buffer[1] = ch;
			// 			memcpy(&buffer[2], params, sizeof(params));
			// 			sendData(conn, buffer, sizeof(buffer));
			// 			break;
			// case 's':
			// case 'S':
			// case 'c':
			// case 'C':
			// case 'g':
			// case 'G':
			// 		params[0]=0;
			// 		params[1]=0;
			// 		memcpy(&buffer[2], params, sizeof(params));
			// 		buffer[1] = ch;
			// 		sendData(conn, buffer, sizeof(buffer));
			// 		break;
			// case 'q':
			// case 'Q':
			// 	quit=1;
			// 	break;
			// case 'v':
			// case 'V':
			// 		printf("Enter yo left and right servo anga(0-180)\n");
			// 		scanf("%d""%d", &params[0], &params[1]);
			// 		flushInput();
			// 		buffer[1] = ch;
			// 		memcpy(&buffer[2], params, sizeof(params));
			// 		sendData(conn, buffer, sizeof(buffer));
			// 		break;

			default:
				printf("BAD COMMAND\n");
				}
			last_time = current_time;
			}
		else {
		printf("Command '%c' ignored (too soon)!\n", ch);
		}
	}
}

	printf("Exiting keyboard thread\n");
	disable_raw_mode();
    /* TODO: Stop the client loop and call EXIT_THREAD */
    stopClient();
    EXIT_THREAD(conn);

    /* END TODO */

    return NULL;
}

/* TODO: #define filenames for the client private key, certificatea,
   CA filename, etc. that you need to create a client */
#define SERVER_NAME "172.20.10.10"
#define CA_CERT_FNAME "signing.pem"
#define PORT_NUM 5001
#define CLIENT_CERT_FNAME "laptop.crt"
#define CLIENT_KEY_FNAME "laptop.key"
#define SERVER_NAME_ON_CERT "mine.com"


/* END TODO */
void connectToServer(const char *serverName, int portNum)
{
    /* TODO: Create a new client */
    createClient(SERVER_NAME, PORT_NUM, 1, CA_CERT_FNAME, SERVER_NAME_ON_CERT, 1, CLIENT_CERT_FNAME, CLIENT_KEY_FNAME, readerThread, writerThread);


    /* END TODO */
}

int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

    networkActive = 1;
    connectToServer(av[1], atoi(av[2]));

    /* TODO: Add in while loop to prevent main from exiting while the
    client loop is running */
	connectToServer(SERVER_NAME,PORT_NUM);
	while(client_is_running());


    /* END TODO */
	printf("\nMAIN exiting\n\n");
}
