# pfs
[![Build Status](https://travis-ci.org/aijunbai/pfs.svg?branch=master)](https://travis-ci.org/aijunbai/pfs)  

A Particle Filtering over Sets Approach to Multi-Object Tracking

This is code release of paper [Intention-Aware Multi-Human Tracking for Human-Robot Interaction via Particle Filtering over Sets](http://aijunbai.github.io/publications/9111-40050-1-PB.pdf), Aijun Bai, Reid Simmons, Manuela Veloso, and Xiaoping Chen, AAAI 2014 Fall Symposium: AI for Human-Robot Interaction (AI-HRI), Arlington, Virginia, United States, November 2014.

Experimental results over PETS2009 S2L1 dataset are available at https://www.youtube.com/watch?v=M2VjS2tMNmg. 

Generated log files can be played by [rcg\_player](https://github.com/aijunbai/rcg_player).

# Dependencies
- libxml2-dev 
- libqt4-dev 
- qt4-qmake 
- libboost-dev 
- libboost-program-options-dev 
- libeigen3-dev 
- libpopt-dev 

# Compiling
- `qmake`  # Running qmake to generate Makefile
- `make -j4` # Building using Makefile

# Running
- `./PETS-S2L1V1.sh`  # Running pfs on PETS-S2L1V1 dataset
- `./TUD-Stadtmitte.sh`  # Running pfs on TUD-Stadtmitte dataset

# Command line interface
Allowed options:

-  --help                                Produce help message
-  --debug arg                           Debug level
-  --framerate arg                       Rate to process depth frames
-  --test [=arg(=1)]                     Test by simulation
-  --task arg                            Task name
-  --bag arg                             Bag files to read from
-  --runto arg                           Run to step
-  --save\_input [=arg(=1)]               Save all inputs (including time)
-  --load\_input [=arg(=human\_tracker.input)]
                                        Load all inputs (including time)
-  --log\_date [=arg(=1)]                 Use date as log name
-  --simulator\_expected\_humans arg       Simulator expected number of humans
-  --simulator\_distance\_threshold [=arg(=1)]
                                        Simulator statistic distance threshold
-  --camera\_calibration [=arg(=MOT-benchmarks/camera.xml)]
                                        Camera calibration file (for benchmark 
                                        testing)
-  --benchmark\_data arg                  Load benchmark detection and ground 
                                        truth data
-  --cropped [=arg(=1)]                  Use cropped data
-  --approximation\_test [=arg(=1)]       Approximation error test
-  --interface [=arg(=1)]                Show benchmark interface
-  --side\_view [=arg(=1)]                Show side-by-side view
-  --report\_threshold arg                Report confidence threshold
-  --seed arg                            Random seed number
-  --num\_particles arg                   Number of particles
-  --position\_kernel\_size arg            Position kernel size
-  --max\_em\_steps arg                    Max EM steps
-  --approaching\_samples arg             Human approaching identification 
                                        samples
-  --false\_rate arg                      False detection rate
-  --missing\_rate arg                    Missing detection rate
-  --false\_density arg                   False detection density
-  --false\_missing\_pruning arg           False-missing pruning threshold
-  --assignments\_pruning arg             Murty pruning ratio threshold
-  --option\_pruning arg                  Option pruning ratio threshold
-  --observation\_proposal\_prob arg       Observation proposal prob
-  --death\_rate arg                      Death rate
-  --refinement\_rate arg                 Refinement rate
-  --human\_area\_min arg                  Min human area
-  --human\_area\_max arg                  Max human area
-  --intention\_mode arg                  Intention initialize mode
-  --hierarchical\_filters [=arg(=1)]     Hierarchical particle filters
-  --mixed\_filters [=arg(=1)]            Mixed particle filters
-  --gaussian\_approximate [=arg(=1)]     Use Gaussian approximate
-  --assignment\_sampling [=arg(=1)]      Use assignment sampling
-  --velocity\_augment [=arg(=1)]         Use velocity augment
-  --detection\_confidence [=arg(=1)]     Use detection confidence
-  --detection\_orientation [=arg(=1)]    Use detection orientation
-  --observation\_error arg               Observation error
-  --threads arg                         Number of threads
