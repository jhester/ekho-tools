# This works on OSX with Homebrew
CC=gcc
BREW=-I/usr/local/include -L/usr/local/lib
GSL=-I/usr/local/Cellar/gsl/1.16/include  -lgsl -lgslcblas
CFLAGS=-g -Wall -pedantic -lm

convert: convert.c
	$(CC) convert.c $(CFLAGS) $(BREW) $(GSL) -o convert
	

clean:
	rm -rf sine  *.dSYM *.o convert
