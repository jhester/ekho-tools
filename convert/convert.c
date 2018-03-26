#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <gsl/gsl_multifit.h>
#include <mach/mach.h>
#include <mach/mach_time.h>


#define __STDC_FORMAT_MACROS

/* Configuration from recording device type / surface accuracy */ 
#define COUNT 256
#define EKHO_N_ARRAY 6
#define NUMPOINTS 65 // CHANGE THIS IF ABOVE CHANGED
#define POLY_DEGREE 3 // This should change based on the harvester, need a more general method, like using splines
#define MAX_VOLTAGE 10.5	
//#define MAX_VOLTAGE 3.28
#define MIN_VOLTAGE 0.0

/* Emulation configuration per Ekho device */
#define REFERENCE_VOLTAGE 3.28
#define ADC_RESOLUTION 4096
#define EMULATION_RESISTANCE 477.8
//#define DAC_DIVISION 1.0
#define DAC_DIVISION 3.2
//#define DAC_DIVISION 10.0 // for TESTING, HACk

/* Profiling and energy harvesting mode binning config */
#define DEFAULT_BIN_P_THRESHOLD 0.001 // 1mW differences

unsigned char buf[8];
double coeff[POLY_DEGREE];
double currentcurve[NUMPOINTS];
double curvedata[2][COUNT];
int indexInAllCurves = 0;
double VOLTAGES[NUMPOINTS] = { 0 };

/* File names */
char *inputfile = NULL;
char *outfile = NULL;

/* Output flags from command line */
int output_siren = 0;
int output_ekho = 0;
int output_gnu = 0;
int mpp_out_opt = 0;
int binning_out_opt = 0;

/* Binning vars */
double bin_threshold = DEFAULT_BIN_P_THRESHOLD;

/* Utility methods */
int polynomialfit(int obs, int degree, double *dx, double *dy, double *store) /* n, p */
{
  gsl_multifit_linear_workspace *ws;
  gsl_matrix *cov, *X;
  gsl_vector *y, *c;
  double chisq;
 
  int i, j;
 
  X = gsl_matrix_alloc(obs, degree);
  y = gsl_vector_alloc(obs);
  c = gsl_vector_alloc(degree);
  cov = gsl_matrix_alloc(degree, degree);
 
  for(i=0; i < obs; i++) {
    gsl_matrix_set(X, i, 0, 1.0);
    for(j=0; j < degree; j++) {
      gsl_matrix_set(X, i, j, pow(dx[i], j));
    }
    gsl_vector_set(y, i, dy[i]);
  }
 
  ws = gsl_multifit_linear_alloc(obs, degree);
  gsl_multifit_linear(X, y, c, cov, &chisq, ws);
 
  /* store result ... */
  for(i=0; i < degree; i++)
  {
    store[i] = gsl_vector_get(c, i);
  }
 
  gsl_multifit_linear_free(ws); 
  gsl_vector_free(c);
  return 1; /* we do not "analyse" the result (cov matrix mainly)
		  to know if the fit is "good" */
}

double polycurve(double voltage, double *coeffs, int degree) {
	double ret = coeffs[1] * voltage + coeffs[0];
	int i;
	for(i=2;i<degree;i++) {
		ret+= coeffs[i] * gsl_pow_int(voltage,i);
	}
	return ret;
}

void sort(double array[2][COUNT]) {
	for (int i = 1; i < COUNT; i++) {
		int j = i;
		while (j > 0 && array[0][j - 1] > array[0][j]) {
			double tempx = array[0][j - 1];
			double tempy = array[1][j - 1];
			// x
			array[0][j - 1] = array[0][j];
			array[0][j] = tempx;
			// y
			array[1][j - 1] = array[1][j];
			array[1][j] = tempy;
			j = j - 1;
		}
	}
}

double clamp_current(double current) {
	return fmin(fmax(current, 0), 0.1f);
}

void create_curves_with_regression_old() {
	double tox[30];
	double toy[30];
	
	// Clear curve first to all zeros
	for (int j = 0; j < NUMPOINTS - 1; j++) {
		currentcurve[j] = 0;
	}

	// Sort curve data cloud on Voltage
	sort(curvedata);

	// Get bounds for polyline
	double leftmostpoint = curvedata[0][0];
	double rightmostpoint = curvedata[0][COUNT - 1];

	// Get poly regression of all points
	polynomialfit(COUNT, POLY_DEGREE, curvedata[0], curvedata[1], coeff);
	int lastIndex = 63;
	int firstIndex = 0;
	for (int j = 0; j < NUMPOINTS - 1; j++) {
		if (VOLTAGES[j] < leftmostpoint) {
			// Get the index where the regression starts in allcurves
			firstIndex = j + 1;
		}
		if (VOLTAGES[j] > leftmostpoint && VOLTAGES[j] < rightmostpoint) {
			currentcurve[j] = clamp_current(polycurve(VOLTAGES[j], coeff, POLY_DEGREE));
		}
		if (VOLTAGES[j] > rightmostpoint) {
			lastIndex = j;
			break;
		}
	}
	
	// Draw a line through left part past point cloud, using points already
	// in curve
	memcpy(tox, &VOLTAGES[firstIndex+1], sizeof(double) * 3);
	memcpy(toy, &currentcurve[firstIndex+1], sizeof(double) * 3);
	polynomialfit(3, 2, tox, toy, coeff); 
	for (int j = 0; j < firstIndex; j++) {
		currentcurve[j] = clamp_current(polycurve(VOLTAGES[j], coeff, 2));
	}
	
	// Draw a line through right part past point cloud, using points already
	// in curve
	// 2-degree since it will most likely curve into zero
	memcpy(tox, &VOLTAGES[lastIndex - 10], sizeof(double) * 10);
	memcpy(toy, &currentcurve[lastIndex - 10],sizeof(double) * 10);	
	polynomialfit(10, 2, tox, toy, coeff);

	// We only want a negative slope between two points, if not negative
	// then render it at zero height
	for (int j = lastIndex; j < NUMPOINTS - 1; j++) {
		if (coeff[1] < 0) {
			currentcurve[j] = clamp_current(polycurve(VOLTAGES[j], coeff, 2));
		} else {
			currentcurve[j] = 0;
		}
	}
}

void create_curves_with_regression() {
	double tox[3];
	double toy[3];
	
	// Clear curve first to all zeros
	for (int j = 0; j < NUMPOINTS - 1; j++) {
		currentcurve[j] = 0;
	}

	// Sort curve data cloud on Voltage
	sort(curvedata);
	
	// Get bounds for polyline
	double leftmostpoint = curvedata[0][0];
	double rightmostpoint = curvedata[0][COUNT - 1];

	// Get poly regression of all points
	polynomialfit(COUNT, POLY_DEGREE, curvedata[0], curvedata[1], coeff);
	int lastIndex = 63;
	int firstIndex = 0;
	for (int j = 0; j < NUMPOINTS - 1; j++) {
		if (VOLTAGES[j] < leftmostpoint) {
			// Get the index where the regression starts in allcurves
			firstIndex = j + 1;
		}
		if (VOLTAGES[j] > leftmostpoint && VOLTAGES[j] < rightmostpoint) {
			currentcurve[j] = clamp_current(polycurve(VOLTAGES[j], coeff, POLY_DEGREE));
		}
		if (VOLTAGES[j] > rightmostpoint) {
			lastIndex = j;
			break;
		}
	}
	
	// Draw a line through left part past point cloud, using points already
	// in curve
	memcpy(tox, &VOLTAGES[firstIndex+1], sizeof(double) * 3);
	memcpy(toy, &currentcurve[firstIndex+1], sizeof(double) * 3);
	polynomialfit(3, 2, tox, toy, coeff); 
	for (int j = 0; j < firstIndex; j++) {
		currentcurve[j] = clamp_current(polycurve(VOLTAGES[j], coeff, 2));
	}
	
	// Draw a line through right part past point cloud, using points already
	// in curve
	// 2-degree since it will most likely curve into zero
	memcpy(tox, &VOLTAGES[lastIndex - 3], sizeof(double) * 3);
	memcpy(toy, &currentcurve[lastIndex - 3],sizeof(double) * 3);	
	polynomialfit(3, 2, tox, toy, coeff);

	// We only want a negative slope between two points, if not negative
	// then render it at zero height
	for (int j = lastIndex; j < NUMPOINTS - 1; j++) {
		if (coeff[1] < 0) {
			currentcurve[j] = clamp_current(polycurve(VOLTAGES[j], coeff, 2));
		} else {
			currentcurve[j] = 0;
		}
	}
	
	// We only want positive currents on back half of the curve, so get highest point in curve
	// and draw a straight line back using highest point and point to the right of it to zero
	int indexforhighpoint=NUMPOINTS-1;
	for(int j = NUMPOINTS-1;j>0;j--) {
		if(currentcurve[j] > currentcurve[indexforhighpoint]) {
			indexforhighpoint = j;
		}
	}
	memcpy(tox, &VOLTAGES[indexforhighpoint], sizeof(double) * 2);
	memcpy(toy, &currentcurve[indexforhighpoint],sizeof(double) * 2);	
	polynomialfit(2, 2, tox, toy, coeff);
	for(int j= 0;j<indexforhighpoint;j++) {
		currentcurve[j] = clamp_current(polycurve(VOLTAGES[j], coeff, 2));
	}
}

void show_help_and_exit() {
	fprintf(stderr, "Convert raw IV data from 'ekho recorder' or elsewhere to various formats for SIREN, gnuplot, Ekho Emulation, and profiling.\n");
	fprintf(stderr, "	Usage ./convert -i <datafile.ivpt> -[segmb] -o <output_file_name>\n");
	fprintf(stderr, "	-h 					Show this help text.\n");
	fprintf(stderr, "	-i <input_file>				The raw IV data file in \"<voltage> <current> <timestamp>\" \n");	
	fprintf(stderr, "	-s 					Output to SIREN format.\n");
	fprintf(stderr, "	-e 					Output to Ekho emulator format.\n");
	fprintf(stderr, "	-g 					Output to GNUplot 5.1 format.\n");	
	fprintf(stderr, "	-m 					Output the MPP trace in Time(s), MPP format.\n");	
	fprintf(stderr, "	-b 					Output the bins for categorizing energy modes\n");		
	fprintf(stderr, "	-t <power> 	        		Threshold difference for bin categorizing energy modes\n");		
	fprintf(stderr, "	-o <filename>				Output filename without extension.\n");	
	exit(0);
}

void command_line_args(int argc, char **argv) {
	int iflag = 0, oflag = 0;
	int c;
	opterr = 0;

	while ((c = getopt (argc, argv, "hi:segmbt:o:")) != -1) {
	    switch (c) {
			case 'h':
				show_help_and_exit();
				break;
			case 'i':
				iflag = 1;
				inputfile = optarg;
				printf("Input file given\n");
				break;
			case 's':
				output_siren = 1;
				printf("SIREN output on\n");
				break;
			case 'e':
				output_ekho = 1;
				printf("Ekho emulator output on\n");
				break;
			case 'g':
				output_gnu = 1;
				printf("GNUplot output on. Requires GNUplot 5.1 to render properly.\n");
				break;
			case 'm':
				mpp_out_opt = 1;
				printf("MPP output on\n");
				break;
			case 'b':
				binning_out_opt = 1;
				break;				
			case 't':
				bin_threshold = atof(optarg);
				break;
			case 'o':
				oflag = 1;
				outfile = optarg;
				printf("Output file given\n");
				break;
			default:
				show_help_and_exit();
	    }
	}
	
	if(!(output_siren || output_ekho || output_gnu)) {
		fprintf(stderr, "***ERROR: Need to specify at least one output format of [seg].\n");
		show_help_and_exit();
	}
	if(!iflag) {
		fprintf(stderr, "***ERROR: Need to specify input file.\n");
		show_help_and_exit();	
	} else {
		// Check file exists
		if( access( inputfile, F_OK ) == -1 ) {
    		// file doesn't exist
    		fprintf(stderr, "***ERROR: Input file does not exist.\n");
    		exit(0);
		}
	}
	if(!oflag) {
		fprintf(stderr, "***ERROR: Need to specify output file name (without extension).\n");
		show_help_and_exit();	
	}
	if(binning_out_opt) {
		printf("Bin by energy harvester mode on for threshold=%.9lf\n", bin_threshold);
	}
}

int main(int argc, char **argv) {
	command_line_args(argc, argv);
	
	// Input file
	FILE *infile = fopen(inputfile, "r");
	// Output file(s)
	FILE *siren_out = NULL;
	FILE *ekho_out = NULL;
	FILE *gnu_out = NULL;
	FILE *mpp_out = NULL;
	
	// Init voltages and other vars
	int points = 0;
	int numcurves = 0;
	double mpp=0.0;
	double timestamp;
	VOLTAGES[0] = 0.0;
	for (int j = 1; j < NUMPOINTS; j++) {
		VOLTAGES[j] = VOLTAGES[j-1] + (MAX_VOLTAGE / (NUMPOINTS - 1));
	}

	// Init files depending on outputs selected
	if(output_siren) {
		char siren_out_file[255];
		strcpy(siren_out_file, outfile);
		strcat (siren_out_file,".siren");
		siren_out = fopen(siren_out_file, "w");
		fprintf(siren_out, "%d\n", EKHO_N_ARRAY); // N
		fprintf(siren_out, "%.9lf,%.9lf\n", MIN_VOLTAGE, MAX_VOLTAGE); // maxv, minv
	}
	if(output_ekho) {
		char ekho_out_file[255];
		strcpy(ekho_out_file, outfile);
		strcat (ekho_out_file,".ekho");
		ekho_out = fopen(ekho_out_file, "w");
	}

	if(output_gnu) {
		char gnu_out_file[255];
		char gnu_out_img_file[255];
		strcpy(gnu_out_file, outfile);
		strcpy(gnu_out_img_file, outfile);
		strcat (gnu_out_file,".gnu");
		strcat (gnu_out_img_file,"_render");
		gnu_out = fopen(gnu_out_file, "w");
		fprintf(gnu_out, "set terminal png  transparent enhanced font \"arial,10\" fontscale 1.0 size 1600, 800\n");
		fprintf(gnu_out, "set output '%s.png'\n", gnu_out_img_file);
		fprintf(gnu_out, "unset border\n");
		fprintf(gnu_out, "set style fill   solid 1.00 noborder\n");
		fprintf(gnu_out, "set dummy u, v\n");
		fprintf(gnu_out, "unset key\n");
		fprintf(gnu_out, "set ticslevel 0\n");
		fprintf(gnu_out, "set view 45,85, 1.145, 1\n");
		fprintf(gnu_out, "set isosamples 75, 75\n");
		fprintf(gnu_out, "unset ztics\n");
		fprintf(gnu_out, "set ylabel \"Time (s)\" \n");
		fprintf(gnu_out, "set xlabel \"Voltage (V)\" rotate parallel\n");
		fprintf(gnu_out, "set title \"\" \n");
		fprintf(gnu_out, "set urange [ -3.14159 : 3.14159 ] noreverse nowriteback\n");
		fprintf(gnu_out, "set vrange [ -3.14159 : 3.14159 ] noreverse nowriteback\n");
		fprintf(gnu_out, "set pm3d depthorder\n");
		fprintf(gnu_out, "set pm3d lighting primary 0.5 specular 0.6\n");
		fprintf(gnu_out, "set palette rgbformulae 8, 9, 7\n");
		fprintf(gnu_out, "splot \"-\" using 3:1:2 with pm3d\n");

		fprintf(gnu_out, "# Time, Current, Voltage\n");
	}
	if(mpp_out_opt) {
		char mpp_out_file[255];
		strcpy(mpp_out_file, outfile);
		strcat (mpp_out_file,".mpp");
		mpp_out = fopen(mpp_out_file, "w");
		fprintf(mpp_out, "# Time(s), MPP\n");
	}

	int n=0;
	/* Start converting the surface a line at a time  */
	while (1) {
		n=fscanf(infile, "%lf	%lf	%lf\n", &curvedata[0][points], &curvedata[1][points], &timestamp);
		
		// End if no more data
		if(n <=0) break;

		// Handle max power point file
		if(mpp_out_opt) {
			double power = curvedata[0][points] * curvedata[1][points];
			if(power > mpp) mpp = power;
		}

		if (points++ == COUNT - 1) {

			// Create new curve (this sorts curve)
			create_curves_with_regression();

			// Pipe curve output to file formats
			if(output_siren) {
				fprintf(siren_out, "%.9lf,", timestamp);	
				for (int j = 0; j < NUMPOINTS; j++) {
					fprintf(siren_out, "%.9lf",currentcurve[j]);
					if(j < NUMPOINTS-1) fprintf(siren_out, ",");
				}
				fprintf(siren_out, "\n");
			}
			if(output_ekho) {
				// Timestamp first
				fprintf(ekho_out, "%.9lf,", timestamp);	
				// Get DAC values for newest curve to play out
				for(int i =0;i<NUMPOINTS;i++) {
					// Convert to dac voltage value Vplay= Vcap + IH * (Rc + Rs)
					// (Rc + Rs) = 470Ω
					// IH = currentcurve[i]
					// This is the amplified voltage
					double dacValueV = VOLTAGES[i] + (EMULATION_RESISTANCE * currentcurve[i]);
					
					// The actual voltage played out pre-amplification
					dacValueV = (dacValueV / DAC_DIVISION);

					// Now convert dac voltage to 12-bit digital 
					uint16_t dacvalue_final = (uint16_t)(dacValueV /  (REFERENCE_VOLTAGE / ((double) ADC_RESOLUTION)));
					dacvalue_final = (dacvalue_final > 4095) ? 4095 : dacvalue_final;

				  	fprintf(ekho_out, "%d", dacvalue_final);	
				  	if(i < NUMPOINTS-1) fprintf(ekho_out, ",");

				}
				
				fprintf(ekho_out, "\n");	
			}
			if(output_gnu) {
				// Print X Y Z C (Time(s) Current(I)  Voltage(V) Color(C)) text for datafile
				for (int j = 0; j < NUMPOINTS; j++) {
					fprintf(gnu_out, "%.9lf	%.9lf	%.9lf\n", timestamp, currentcurve[j], VOLTAGES[j]);	
				}
				fprintf(gnu_out, "\n");
			}
			
			// For profiler we need to capture the different energy harvesting modes for some threshold
			if(binning_out_opt) {

			}

			// Handle max power point file
			if(mpp_out_opt) {
				fprintf(mpp_out, "%.9lf %.9lf\n", timestamp, mpp);
				mpp = 0.0;
			}

			// Prepare for next curve
			points = 0;
			numcurves++;
		}
	}

	// Close all files
	if(output_siren) {
		fclose(siren_out);
	}
	if(output_ekho) {
		fclose(ekho_out);
	}
	if(output_gnu) {
		fclose(gnu_out);
	}
	if(mpp_out_opt) {
		fclose(mpp_out);	
	}
	fclose(infile);
	return 0;
}
