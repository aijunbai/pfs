#!/bin/bash

CPUS=`cat /proc/cpuinfo | grep processor | wc -l`

./tracker.sh --camera_calibration MOT-benchmarks/cparsxml/View_001.xml --benchmark_data MOT-benchmarks/data/det/PETS2009/PETS2009-S2L1-c1-det.xml MOT-benchmarks/data/gt/PETS2009/PETS2009-S2L1.xml --framerate 7 --simulator_distance_threshold 1.0 --intention_mode -1 --interface --side_view --threads $CPUS $*
