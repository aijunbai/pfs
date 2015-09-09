#!/bin/bash

./tracker.sh -C MOT-benchmarks/data/gt/TUD/TUD-Stadtmitte-calib.xml -B MOT-benchmarks/data/det/TUD/TUD-stadtmitte-det.xml MOT-benchmarks/data/gt/TUD/TUD-Stadtmitte.xml -I -1 -F 25 -# 1.0 $*
