 

#include "pb_omni_pid_pursuit_controller/pid.hpp"

PID::PID(double dt, double max, double min, double kp, double kd, double ki)
: dt_(dt), max_(max), min_(min), kp_(kp), kd_(kd), ki_(ki), pre_error_(0), integral_(0)
{
}

double PID::calculate(double set_point, double pv)
{
  // Calculate error
  double error = set_point - pv;

  // Proportional term
  double p_out = kp_ * error;

  // Integral term (clamp BEFORE computing i_out so the current output already
  // reflects the anti-windup limit, instead of lagging one cycle behind).
  integral_ += error * dt_;
  if (integral_ > integral_limit_) {
    integral_ = integral_limit_;
  } else if (integral_ < -integral_limit_) {
    integral_ = -integral_limit_;
  }
  double i_out = ki_ * integral_;

  // Derivative term
  double derivative = (error - pre_error_) / dt_;
  double d_out = kd_ * derivative;

  // Calculate total output
  double output = p_out + i_out + d_out;

  // Restrict to max/min
  if (output > max_)
    output = max_;
  else if (output < min_)
    output = min_;

  // Save error to previous error
  pre_error_ = error;

  return output;
}

void PID::setSumError(double sum_error) { integral_ = sum_error; }

void PID::setSumLimit(double limit)
{
  if (limit > 0.0) {
    integral_limit_ = limit;
  }
}

PID::~PID() {}
