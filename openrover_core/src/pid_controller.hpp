#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <string>

double clamp(double value, double min, double max);

class PIDController
{
public:
  const double proportional_gain;
  const double integral_gain;
  const double differential_gain;
  const double windup_limit;


protected:
  rclcpp::Time last_time;
  double target;
  double error_integral;
  double control_value;
  double last_error;
  const std::string output_file;
  std::ofstream TEST_FILE;

public:
  PIDController(
    double proportional_gain, double integral_gain, double differential_gain, double windup_limit,
    const rclcpp::Time & time_zero);
  PIDController(
    double proportional_gain, double integral_gain, double differential_gain, double windup_limit,
    const rclcpp::Time & time_zero, const std::string output_file);

  /// Chooses a new target value that the controller is aiming for
  /// @param new_target the new set point
  void set_target(double new_target) { target = new_target; }

  /// Drives the controller forward one step. This should be called frequently,
  /// with the return value being passed into the system being controlled
  /// @param now The current time
  /// @param measured_value Observed output value during this time period
  /// @return New control value
  double step(const rclcpp::Time & now, double measured_value);
};
