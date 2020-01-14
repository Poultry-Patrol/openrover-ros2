#include "filters.hpp"

Filters::Filters()
  : MAX_ROCOF(2800.0), // 3750.0)
    frequencies({0.0, 0.0, 0.0})
{}

void Filters::set_rocof(double max_rocof)
{
  MAX_ROCOF = max_rocof;
}

void

double Filters::rocof_window(std::vector<double> frequencies, double dt) {
  auto rocof = (frequencies[0] - frequencies[1]) / dt;
  if (rocof > MAX_ROCOF) {
    return frequencies[1] + (MAX_ROCOF * dt);
  }
  else if (rocof < -MAX_ROCOF) {
    return frequencies[1] - (MAX_ROCOF * dt);
  }
  else
    return frequencies[0];
}

double Filters::low_pass(std::vector<double> frequencies) {
  return 0.25 * frequencies[0] + 0.5 * frequencies[1] + 0.25 * frequencies[2];
}