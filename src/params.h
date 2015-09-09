/*
 * params.h
 *
 *  Created on: Feb 8, 2014
 *      Author: baj
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include <cstdlib>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <eigen3/Eigen/Dense>

class Params
{
  Params();
  virtual ~Params();

public:
  static Params &ins();

  double text_margin;

  double false_rate;
  double missing_rate;
  double false_density;
  double false_missing_pruning;
  double assignments_pruning;
  double option_pruning;
  double observation_proposal_prob;
  double death_rate;
  double refinement_rate;
  double human_area_min;
  double human_area_max;

  double observation_error;
  double max_speed;
  double sigma_power;
  double view_length;
  double view_width;

  double sleep_duration;

  uint trajectory_size;
  uint max_humans;
  uint max_terms;
  uint seeding;
  uint num_particles;
  uint position_kernel_size;
  double change_intention_rate;
  uint max_em_steps;

  bool gaussian_approximate;
  bool velocity_augment;
  bool detection_confidence;
  bool hierarchical_filters;
  bool mixed_filters;
  bool assignment_sampling;

  int intention_mode;
  uint threads;

  int debug_level;
  double frame_rate;
  std::vector<std::string> bag_names;
  std::string task_name;
  bool simulation_testing;
  bool save_input;
  std::string load_input;
  bool log_date;
  int runto_step;
  double simulator_expected_humans;
  double CLEAR_distance_threshold;
  std::string camera_calibration;
  std::vector<std::string> benchmark_data;
  bool cropped;
  bool approximation_test;

  bool interface;
  bool side_view;
  double report_threshold;

  bool Parse(int argc, char** argv, bool verbose = false);
  void Dump();
};

#endif /* PARAMS_H_ */

