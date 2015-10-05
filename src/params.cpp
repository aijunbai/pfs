/*
 * params.cpp
 *
 *  Created on: Feb 8, 2014
 *      Author: baj
 */

#include <params.h>
#include "logger.h"
#include "common.h"

using namespace std;
using namespace boost::program_options;

Params::Params()
{
  text_margin = -7.5;

  false_rate = 2.0;
  missing_rate = 2.0;
  false_density = 0.5;
  false_missing_pruning = 0.001;
  assignments_pruning = 0.1;
  option_pruning = 0.001;
  observation_proposal_prob = 1.0;
  observation_error = 0.25;
  death_rate = 0.02;
  refinement_rate = 0.0;
  human_area_min = 0.5;
  human_area_max = 2.5;

  max_speed = 2.0;
  sigma_power = 2.0;
  view_length = 12.0;
  view_width = 57.0;

  sleep_duration = 0.2;

  trajectory_size = 500;
  max_humans = 5;
  max_terms = 1000;
  seed = 0;
  num_particles = 128;
  position_kernel_size = 256;
  change_intention_rate = 0.5;
  max_em_steps = 1;
  approaching_samples = 10;

  gaussian_approximate = false;
  velocity_augment = false;
  detection_confidence = false;
  detection_orientation = false;
  hierarchical_filters = false;
  mixed_filters = false;
  assignment_sampling = false;

  intention_mode = 0;
  threads = 2;

  debug_level = 1;
  frame_rate = 5.0;
  task_name = "OpenAreaTask";
  simulation_test = false;
  save_input = false;
  log_date = false;
  runto_step = -1;
  simulator_expected_humans = 3;
  CLEAR_distance_threshold = -1.0;
  cropped = true;
  approximation_test = false;
  interface = false;
  side_view = false;
  report_threshold = 0.0;
}

Params::~Params()
{
}

Params &Params::ins()
{
  static Params params;
  return params;
}

bool Params::Parse(int argc, char** argv, bool verbose)
{
  options_description desc("Allowed options");

  desc.add_options()
      ("help", "Produce help message")
      ("debug", value<int>(&debug_level), "Debug level")
      ("framerate", value<double>(&frame_rate),
          "Rate to process depth frames")
      ("test", value<bool>(&simulation_test)->implicit_value(true),
          "Test by simulation")
      ("task", value<string>(&task_name), "Task name")
      ("bag", value<vector<string> >(&bag_names)->multitoken(),
          "Bag files to read from")
      ("runto", value<int>(&runto_step), "Run to step")
      ("save_input", value<bool>(&save_input)->implicit_value(true),
          "Save all inputs (including time)")
      ("load_input",
          value<string>(&load_input)->implicit_value(
          string(DEFAULT_LOG_NAME) + ".input"),
          "Load all inputs (including time)")
      ("log_date",
          value<bool>(&log_date)->implicit_value(true),
          "Use date as log name")
      ("simulator_expected_humans", value<double>(&simulator_expected_humans),
          "Simulator expected number of humans")
      ("simulator_distance_threshold",
          value<double>(&CLEAR_distance_threshold)->implicit_value(1.0),
          "Simulator statistic distance threshold")
      ("camera_calibration",
          value<string>(&camera_calibration)->implicit_value("MOT-benchmarks/camera.xml"),
           "Camera calibration file (for benchmark testing)")
      ("benchmark_data", value<vector<string> >(&benchmark_data)->multitoken(),
           "Load benchmark detection and ground truth data")
      ("cropped",
            value<bool>(&cropped)->implicit_value(true),
            "Use cropped data")
      ("approximation_test",
            value<bool>(&approximation_test)->implicit_value(true),
            "Approximation error test")
      ("interface",
           value<bool>(&interface)->implicit_value(true),
           "Show benchmark interface")
      ("side_view",
            value<bool>(&side_view)->implicit_value(true),
            "Show side-by-side view")
      ("report_threshold",
           value<double>(&report_threshold),
           "Report confidence threshold")
      ("seed", value<uint>(&seed),
          "Random seed number")
      ("num_particles",
          value<uint>(&num_particles),
          "Number of particles")
      ("position_kernel_size",
          value<uint>(&position_kernel_size),
          "Position kernel size")
      ("max_em_steps",
          value<uint>(&max_em_steps),
          "Max EM steps")
      ("approaching_samples",
          value<uint>(&approaching_samples),
          "Human approaching identification samples")
      ("false_rate",
          value<double>(&false_rate),
          "False detection rate")
      ("missing_rate",
          value<double>(&missing_rate),
          "Missing detection rate")
      ("false_density",
          value<double>(&false_density),
          "False detection density")
      ("false_missing_pruning",
          value<double>(&false_missing_pruning),
          "False-missing pruning threshold")
      ("assignments_pruning",
          value<double>(&assignments_pruning),
          "Murty pruning ratio threshold")
      ("option_pruning",
          value<double>(&option_pruning),
          "Option pruning ratio threshold")
      ("observation_proposal_prob",
          value<double>(&observation_proposal_prob),
          "Observation proposal prob")
      ("death_rate",
          value<double>(&death_rate),
          "Death rate")
      ("refinement_rate",
          value<double>(&refinement_rate),
          "Refinement rate")
      ("human_area_min",
          value<double>(&human_area_min),
          "Min human area")
      ("human_area_max",
          value<double>(&human_area_max),
          "Max human area")
      ("intention_mode",
          value<int>(&intention_mode),
          "Intention initialize mode")
      ("hierarchical_filters",
          value<bool>(&hierarchical_filters)->implicit_value(true),
          "Hierarchical particle filters")
      ("mixed_filters",
          value<bool>(&mixed_filters)->implicit_value(true),
          "Mixed particle filters")
      ("gaussian_approximate",
          value<bool>(&gaussian_approximate)->implicit_value(true),
          "Use Gaussian approximate")
      ("assignment_sampling",
          value<bool>(&assignment_sampling)->implicit_value(true),
          "Use assignment sampling")
      ("velocity_augment",
          value<bool>(&velocity_augment)->implicit_value(true),
          "Use velocity augment")
      ("detection_confidence",
          value<bool>(&detection_confidence)->implicit_value(true),
          "Use detection confidence")
      ("detection_orientation",
          value<bool>(&detection_orientation)->implicit_value(true),
          "Use detection orientation")
      ("observation_error",
          value<double>(&observation_error),
          "Observation error")
      ("threads",
          value<uint>(&threads),
          "Number of threads");

  try {
    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    if ( vm.count("help")  ) {
      TerminalLogger::log() << desc << endl;
      return 0;
    }
    notify(vm);
  }
  catch(error& e) {
    cerr << "ERROR: " << e.what() << endl << endl;
    cerr << desc << endl;
    return false;
  }

  const double sleep_buffer = 0.03;
  sleep_duration = 1.0 / frame_rate - sleep_buffer;

  if (simulation_test || !benchmark_data.empty()) {
    sleep_duration += sleep_buffer;
  }
  else if (sleep_duration < 0.01) {
    sleep_duration = 0.01;
  }

  if (verbose) {
    Dump();
  }

  return true;
}

void Params::Dump()
{
  if (debug_level > 0) {
    PRINT_VALUE("Options");

    PRINT_VALUE(debug_level);
    PRINT_VALUE(frame_rate);
    PRINT_VALUE(task_name);
    PRINT_VALUE(simulation_test);
    PRINT_VALUE(save_input);
    PRINT_VALUE(load_input);
    PRINT_VALUE(log_date);
    PRINT_VALUE(runto_step);
    PRINT_VALUE(simulator_expected_humans);
    PRINT_VALUE(CLEAR_distance_threshold);
    PRINT_VALUE(cropped);
    PRINT_VALUE(approximation_test);
    PRINT_VALUE(interface);
    PRINT_VALUE(side_view);
    PRINT_VALUE(report_threshold);

    PRINT_VALUE(false_rate);
    PRINT_VALUE(missing_rate);
    PRINT_VALUE(false_density);
    PRINT_VALUE(false_missing_pruning);
    PRINT_VALUE(assignments_pruning);
    PRINT_VALUE(option_pruning);
    PRINT_VALUE(observation_proposal_prob);
    PRINT_VALUE(death_rate);
    PRINT_VALUE(refinement_rate);
    PRINT_VALUE(human_area_min);
    PRINT_VALUE(human_area_max);

    PRINT_VALUE(observation_error);
    PRINT_VALUE(max_speed);
    PRINT_VALUE(sigma_power);
    PRINT_VALUE(view_length);
    PRINT_VALUE(view_width);

    PRINT_VALUE(sleep_duration);

    PRINT_VALUE(trajectory_size);
    PRINT_VALUE(max_humans);
    PRINT_VALUE(max_terms);
    PRINT_VALUE(seed);
    PRINT_VALUE(num_particles);
    PRINT_VALUE(position_kernel_size);
    PRINT_VALUE(change_intention_rate);
    PRINT_VALUE(max_em_steps);
    PRINT_VALUE(approaching_samples);
    PRINT_VALUE(threads);

    PRINT_VALUE(gaussian_approximate);
    PRINT_VALUE(velocity_augment);
    PRINT_VALUE(detection_confidence);
    PRINT_VALUE(detection_orientation);
    PRINT_VALUE(hierarchical_filters);
    PRINT_VALUE(mixed_filters);
    PRINT_VALUE(intention_mode);
    PRINT_VALUE(assignment_sampling);

    PRINT_VALUE("\n");
  }
}

