#!/bin/sh
P=`pwd`
cd chargen
ls *.c | grep -v test.c | while read l; do arm-none-eabi-gcc -c $l;done
arm-none-eabi-ar -mc libchargen.a *.o
mv libchargen.a $P
#cp build/atom*.o ../../..
