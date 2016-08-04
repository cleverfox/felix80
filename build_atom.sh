#!/bin/sh
P=`pwd`
cd atomthreads/ports/cortex-m
gmake OPENCM3_DIR=$P/libopencm3
cp build/atom*.o ../../..
