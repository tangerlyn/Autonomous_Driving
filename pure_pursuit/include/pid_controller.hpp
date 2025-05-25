#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

class PIDController {
public:
  PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

  double update(double error, double dt) {
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

  void set_gains(double kp, double ki, double kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
  }

private:
  double kp_, ki_, kd_;
  double prev_error_, integral_;
};

#endif

