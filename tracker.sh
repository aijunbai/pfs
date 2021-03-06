#!/bin/bash

ulimit -c unlimited

rm -f core*
rm -f valgrind.log vgcore.* hs_err_pid*.log

DIR=`pwd`
mkdir -p logs/human_tracker/
cd logs/human_tracker/
rm -fr bak/
mkdir -p bak/intentions
mkdir -p intentions
mv human_tracker* bak/
mv intentions/* bak/intentions/
cd $DIR

time ./pfs $@ | tee human_tracker.output

