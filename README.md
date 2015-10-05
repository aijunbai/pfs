# pfs
A Particle Filtering over Sets Approach for Multi-Object Tracking

Experimental results over PETS2009 S2L1 dataset are available at 
https://www.youtube.com/watch?v=M2VjS2tMNmg. 

Generated log files can be played by rcg_player (https://github.com/aijunbai/rcg_player).

## Usage
### Building
- qmake  # Running qmake to generate Makefile
- make -j4  # Building using Makefile

### Running
- ./PETS-S2L1V1.sh  # Running pfs on PETS-S2L1V1 dataset
- ./TUD-Stadtmitte.sh  # Running pfs on TUD-Stadtmitte dataset
- ./PETS-S2L1V1.sh -i  # Running pfs on PETS-S2L1V1 dataset with one-view GUI
- ./TUD-Stadtmitte.sh -i # Running pfs on TUD-Stadtmitte dataset with one-view GUI
- ./PETS-S2L1V1.sh -i -X  # Running pfs on PETS-S2L1V1 dataset with four-view GUI
- ./TUD-Stadtmitte.sh -i -X # Running pfs on TUD-Stadtmitte dataset with four-view GUI

### Command line interface
Allowed options:

-  -h [ --help ]                         Produce help message
-  -d [ --debug ] arg                    Debug level
-  -F [ --framerate ] arg                Rate to process depth frames
-  -T [ --testing ] [=arg(=1)]           Testing by simulation
-  -t [ --task ] arg                     Task name
-  -b [ --bag ] arg                      Bag files to read from
-  -r [ --runto ] arg                    Run to step
-  -S [ --save_input ] [=arg(=1)]        Save all inputs (including time)
-  -L [ --load_input ] [=arg(=human_tracker.input)]
-                                        Load all inputs (including time)
-  -l [ --log_date ] [=arg(=1)]          Use date as log name
-  -x [ --simulator_expected_humans ] arg
-                                        Simulator expected number of humans
-  -# [ --simulator_distance_threshold ] [=arg(=1)]
-                                        Simulator statistic distance threshold
-  -C [ --camera_calibration ] [=arg(=MOT-benchmarks/camera.xml)]
-                                        Camera calibration file (for benchmark 
-                                        testing)
-  -B [ --benchmark_data ] arg           Load benchmark detection and ground 
-                                        truth data
-  -Z [ --cropped ] [=arg(=1)]           Use cropped data
-  -y [ --approximation_test ] [=arg(=1)]
-                                        Approximation error test
-  -i [ --interface ] [=arg(=1)]         Show benchmark interface
-  -X [ --side_view ] [=arg(=1)]         Show side-by-side view
-  -R [ --report_threshold ] arg         Report confidence threshold
-  -s [ --seeding ] arg                  Random seeding number
-  -n [ --num_particles ] arg            Number of particles
-  -k [ --position_kernel_size ] arg     Position kernel size
-  -E [ --max_em_steps ] arg             Max EM steps
-  -f [ --false_rate ] arg               False detection rate
-  -m [ --missing_rate ] arg             Missing detection rate
-  -g [ --false_density ] arg            False detection density
-  -p [ --false_missing_pruning ] arg    False-missing pruning threshold
-  -P [ --assignments_pruning ] arg      Murty pruning ratio threshold
-  -o [ --option_pruning ] arg           Option pruning ratio threshold
-  -O [ --observation_proposal_prob ] arg
-                                        Observation proposal prob
-  -D [ --death_rate ] arg               Death rate
-  -j [ --refinement_rate ] arg          Refinement rate
-  -a [ --human_area_min ] arg           Min human area
-  -A [ --human_area_max ] arg           Max human area
-  -I [ --intention_mode ] arg           Intention initialize mode
-  -H [ --hierarchical_filters ] [=arg(=1)]
-                                        Hierarchical particle filters
-  -M [ --mixed_filters ] [=arg(=1)]     Mixed particle filters
-  -N [ --gaussian_approximate ] [=arg(=1)]
-                                        Use Gaussian approximate
-  -K [ --assignment_sampling ] [=arg(=1)]
-                                        Use assignment sampling
-  -v [ --velocity_augment ] [=arg(=1)]  Use velocity augment
-  -c [ --detection_confidence ] [=arg(=1)]
-                                        Use detection confidence
-  -z [ --observation_error ] arg        Observation error
-  -@ [ --threads ] arg                  Number of threads
