CC=gcc
GL=-framework Foundation -framework glut -framework opengl
# added -Wno-deprecated because on Mac 10.10 GLUT functions are all deprecated
CFLAGS=-g -Wall -pedantic -lm -Wno-deprecated
BREW=-I/usr/local/include -L/usr/local/lib
GSL=-I/usr/local/Cellar/gsl/1.16/include  -lgsl -lgslcblas

record: record.c
	$(CC) record.c $(CFLAGS) $(GL) -o record

clean:
	rm -rf record *.o *.dSYM

