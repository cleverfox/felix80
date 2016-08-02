#!/bin/sh
cd atomthreads/ports/cortex-m
gmake
cp build/atom*.o ../../..
