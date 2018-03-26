#include "record.h"
#include <unistd.h>
//#define DEBUG

#define SENSE_RESISTANCE 10.2
#define COUNT 512
#define NUMPOINTS 65

#define MOBILE_EKHO_V2
//#define MOBILE_EKHO
//#define DESKTOP_EKHO


#ifdef MOBILE_EKHO_V2
#define CS_GAIN 100
#define VS_GAIN 4
#endif

#ifdef DESKTOP_EKHO
#define CS_GAIN 10
#define VS_GAIN 1
#endif

#define READSIZE 6
unsigned char buf[READSIZE];
int port;
double curvedata[2][COUNT];
GLfloat color1[3] = {1.0,1.0,1.0};

/* Serial init and configuration */
void init_serial(char *serial_port_str) {
	struct termios settings;
	// Open the serial port
	port = open(serial_port_str, O_RDWR);
	if (port < 0) {
		fprintf(stderr, "Unable to open %s\n", serial_port_str);
		exit(0);
	}

	// Configure the port
	tcgetattr(port, &settings);
	cfmakeraw(&settings);
	tcsetattr(port, TCSANOW, &settings);	
}

void draw_data()
{
	int i;
	/*  Draw something here */
	glPointSize(4.0);
	glColor3fv( color1 );	
	glBegin(GL_POINTS);
	glVertex2f(0.1, 0.001);
	for(i=0;i<COUNT;i++) {
		glVertex2f(curvedata[0][i], curvedata[1][i]);
	}	
	glEnd();
}

FILE *outfile;
double timestamp=0.0;
uint64_t start;
uint64_t end;
uint64_t elapsed;
uint64_t elapsedNano;
static mach_timebase_info_data_t    sTimebaseInfo;	
void idle_impl()
{
	int n=0;
	int points = 0;
   	while (1) {
		memset(buf, 0, sizeof(unsigned char) * READSIZE);
		n = read(port, &buf[0], READSIZE);
		/*if (n < READSIZE) {
			fprintf(stderr, "error reading from port, read %d\n", n);
			continue;
		}*/
		//unsigned int check1 = ((buf[6] << 8) + buf[5]);
		unsigned int check = ((buf[5] << 8) + buf[4]);
		if (check == 0xFFFF) {
			
			double current = (((buf[1]) << 8) + buf[0]) * (3.28f / 4096.0f);
			double voltage = (((buf[3]) << 8) + buf[2]) * (3.28f / 4096.0f);
			current = current / ( SENSE_RESISTANCE * CS_GAIN );
			voltage = voltage * VS_GAIN;

#ifdef DEBUG
			printf("%lf,%lf\n", voltage, current);
#endif

			curvedata[0][points] = voltage;
			curvedata[1][points] = current;
			
			end = mach_absolute_time();
			elapsed = end - start;
			elapsedNano = elapsed * sTimebaseInfo.numer / sTimebaseInfo.denom;
			// Pipe timestamp to both files
			timestamp +=  elapsedNano / 1000000000.0;
			fprintf(outfile, "%.9lf	%.9lf	%.9lf\n", voltage, current, timestamp);
			if (points++ == COUNT - 1) {
				// Update display
				break;
			}
		} else {
			n = read(port, buf, 1);
			if (n < 1) {
				fprintf(stderr, "error over-reading from port\n");
				break;
			}
		}
		start = mach_absolute_time();
	}
}


void show_help_and_exit() {
	fprintf(stderr, "Records IV surfaces with visual interface. Connect to Ekho record device and saves RAW IV point data to a file.\n");
	fprintf(stderr, "	Usage ./record -s <serial_port>  -o <output_file_name>\n");
	fprintf(stderr, "	-h 					Show this help text.\n");
	fprintf(stderr, "	-s <serial_port>			The /dev/cu.***** port connected to Ekho recorder. \n");	
	fprintf(stderr, "	-o <filename>				Output filename without extension.\n");	
	exit(0);
}

int main ( int argc, char** argv )   // Create Main Function For Bringing It All Together
{
	// Parse cmd line args
	int iflag = 0, oflag = 0;
	int c;
	opterr = 0;
	char *outfilestring = NULL;
	char *serialportstring = NULL;

	while ((c = getopt (argc, argv, "hs:o:")) != -1) {
	    switch (c) {
			case 'h':
				show_help_and_exit();
				break;
			case 's':
				iflag = 1;
				serialportstring = optarg;
				printf("Serial port given\n");
				break;
			case 'o':
				oflag = 1;
				outfilestring = optarg;
				printf("Output file given\n");
				break;
			default:
				show_help_and_exit();
	    }
	}

	// Now get to work

	if(!iflag) {
		fprintf(stderr, "***ERROR: Need to specify serial port.\n");
		show_help_and_exit();	
	}

	if(!oflag) {
		fprintf(stdout, "Saving data to 'rawdata.ivp'\n");
		outfile = fopen("rawdata.ivp", "w");
	} else {
		outfile = fopen(outfilestring, "w");
	}
	
	if ( sTimebaseInfo.denom == 0 ) {
		(void) mach_timebase_info(&sTimebaseInfo);
	}
	start = mach_absolute_time();
	if ( sTimebaseInfo.denom == 0 ) {
		(void) mach_timebase_info(&sTimebaseInfo);
	}
	init_serial(serialportstring);
	//int n = write(port, buf, 1);
	//if (n < 1) {
	//	fprintf(stderr, "error sending start command to Ekho device\n");
	//	exit(0);
	//}
	run(argc, argv, 4.0, 0.001, 1000, 500);

  return 0;
}

