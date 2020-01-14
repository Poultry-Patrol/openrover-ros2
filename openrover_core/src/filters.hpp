#pragma once
#include <vector>

class Filters {
  // Variables
  private:
  double MAX_ROCOF;

  // Member Functions
  public:
  Filters();
  void set_rocof(double max_rocof);
  double rocof_window(std::vector<double> frequencies, double dt);
  std::vector<double> frequencies;
  std::vector<double> effort;
  double filter_direction(double)
  double low_pass(std::vector<double> frequencies);
};

