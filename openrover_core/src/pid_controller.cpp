//
// Created by snuc on 6/19/19.
//

#include "pid_controller.hpp"
#include <sstream>

double clamp(double value, double min, double max) {
  assert(min <= max);
  return (value < min) ? min : (max < value) ? max : value;
}

PIDController::PIDController(
  double proportional_gain, double integral_gain, double differential_gain, double windup_limit,
  const rclcpp::Time &time_zero)
  : proportional_gain(proportional_gain),
    integral_gain(integral_gain),
    differential_gain(differential_gain),
    windup_limit(windup_limit),
    last_time(time_zero),
    output_file(""),
    target(0.0),
    error_integral(0.0),
    control_value(0.0),
    last_error(0.0) {
  assert(proportional_gain >= 0);
  assert(integral_gain >= 0);
  assert(windup_limit > 0);
  assert(proportional_gain || integral_gain);
}

PIDController::PIDController(
  double proportional_gain, double integral_gain, double differential_gain, double windup_limit,
  const rclcpp::Time &time_zero, const std::string output_file)
  : proportional_gain(proportional_gain),
    integral_gain(integral_gain),
    differential_gain(differential_gain),
    windup_limit(windup_limit),
    last_time(time_zero),
    output_file(output_file),
    target(0.0),
    error_integral(0.0),
    control_value(0.0),
    last_error(0.0) {
  assert(proportional_gain >= 0);
  assert(integral_gain >= 0);
  assert(windup_limit > 0);
  assert(proportional_gain || integral_gain);

  TEST_FILE.open(output_file, std::ofstream::out | std::ofstream::app);
  TEST_FILE << "measured_value,control_value,target,kp,ki,kd,error,error_integral,error_differential,interval,time\n";
}

double PIDController::step(const rclcpp::Time &now, double measured_value) {
  assert(last_time < now);
  auto interval = now - last_time;
  auto error = target - measured_value;
  error_integral += error * interval.seconds();
  error_integral = clamp(error_integral, -windup_limit, +windup_limit);
  last_time = now;
  control_value = error * proportional_gain + error_integral * integral_gain + differential_gain * (error - last_error) / interval.seconds();
  if (TEST_FILE.is_open()){
    TEST_FILE << measured_value << ",";
    TEST_FILE << control_value << ",";
    TEST_FILE << target << ",";
    TEST_FILE << proportional_gain << ",";
    TEST_FILE << integral_gain << ",";
    TEST_FILE << differential_gain << ",";
    TEST_FILE << error << ",";
    TEST_FILE << error_integral << ",";
    TEST_FILE << (error - last_error) / interval.seconds() << ",";
    TEST_FILE << interval.seconds() <<  ",";
    TEST_FILE << now.seconds() << "\n";
  }

  last_error = error;
  return control_value;
}
