# pfs
[![Build Status](https://travis-ci.org/aijunbai/pfs.svg?branch=master)](https://travis-ci.org/aijunbai/pfs)  

A Particle Filtering over Sets Approach to Multi-Object Tracking

This is code release of paper [Intention-Aware Multi-Human Tracking for Human-Robot Interaction via Particle Filtering over Sets](http://aijunbai.github.io/publications/AAAI14-Bai.pdf), Aijun Bai, Reid Simmons, Manuela Veloso, and Xiaoping Chen, AAAI 2014 Fall Symposium: AI for Human-Robot Interaction (AI-HRI), Arlington, Virginia, United States, November 2014.

Experimental results over PETS2009 S2L1 dataset are available at [https://www.youtube.com/watch?v=M2VjS2tMNmg](https://www.youtube.com/watch?v=M2VjS2tMNmg).

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

# Options
Allowed options of `pfs`:
```
  --help                                Produce help message
  --debug arg                           Debug level
  --framerate arg                       Rate to process depth frames
  --test [=arg(=1)]                     Test by simulation
  --task arg                            Task name
  --bag arg                             Bag files to read from
  --runto arg                           Run to step
  --save_input [=arg(=1)]               Save all inputs (including time)
  --load_input [=arg(=human_tracker.input)]
                                        Load all inputs (including time)
  --log_date [=arg(=1)]                 Use date as log name
  --simulator_expected_humans arg       Simulator expected number of humans
  --simulator_distance_threshold [=arg(=1)]
                                        Simulator statistic distance threshold
  --camera_calibration [=arg(=MOT-benchmarks/camera.xml)]
                                        Camera calibration file (for benchmark 
                                        testing)
  --benchmark_data arg                  Load benchmark detection and ground 
                                        truth data
  --cropped [=arg(=1)]                  Use cropped data
  --approximation_test [=arg(=1)]       Approximation error test
  --interface [=arg(=1)]                Show benchmark interface
  --side_view [=arg(=1)]                Show side-by-side view
  --report_threshold arg                Report confidence threshold
  --seed arg                            Random seed number
  --num_particles arg                   Number of particles
  --position_kernel_size arg            Position kernel size
  --max_em_steps arg                    Max EM steps
  --approaching_samples arg             Human approaching identification 
                                        samples
  --false_rate arg                      False detection rate
  --missing_rate arg                    Missing detection rate
  --false_density arg                   False detection density
  --false_missing_pruning arg           False-missing pruning threshold
  --assignments_pruning arg             Murty pruning ratio threshold
  --option_pruning arg                  Option pruning ratio threshold
  --observation_proposal_prob arg       Observation proposal prob
  --death_rate arg                      Death rate
  --refinement_rate arg                 Refinement rate
  --human_area_min arg                  Min human area
  --human_area_max arg                  Max human area
  --intention_mode arg                  Intention initialize mode
  --hierarchical_filters [=arg(=1)]     Hierarchical particle filters
  --mixed_filters [=arg(=1)]            Mixed particle filters
  --gaussian_approximate [=arg(=1)]     Use Gaussian approximate
  --assignment_sampling [=arg(=1)]      Use assignment sampling
  --velocity_augment [=arg(=1)]         Use velocity augment
  --detection_confidence [=arg(=1)]     Use detection confidence
  --detection_orientation [=arg(=1)]    Use detection orientation
  --observation_error arg               Observation error
  --threads arg                         Number of threads
```

