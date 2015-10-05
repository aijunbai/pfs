#!/bin/bash

CPUS=`cat /proc/cpuinfo | grep processor | wc -l`

./tracker.sh --camera_calibration MOT-benchmarks/data/gt/TUD/TUD-Stadtmitte-calib.xml --benchmark_data MOT-benchmarks/data/det/TUD/TUD-stadtmitte-det.xml MOT-benchmarks/data/gt/TUD/TUD-Stadtmitte.xml --intention_mode -1 --framerate 25 --simulator_distance_threshold 1.0  --interface --side_view --threads $CPUS $*
