#pragma once

class PID {
 public:
  struct Gains {
    double kP,kI,kD;
  };

  PID(Gains gains) : _gains(gains) {} // P=0.1-0.3, I=0, D=0

  void setWrap(double range) {
    _wrapRange = range;
  }
  
  double previousOutput = 0;
  double previousError = 0;
  double sum = 0;

  double calculate(double input, double goal, double dt) {
    double error = wrap(goal - input);
    double derror = (error - previousError) / dt;
    sum += error * dt;

    double output = _gains.kP * error + PID::_gains.kI * sum + _gains.kD * derror;

    previousError = error;
    previousOutput = output;

    return output;
  }

 private:
  Gains _gains;
  double _wrapRange = -1;

  double wrap(double val) {
    if (_wrapRange > 0) {
      val = std::fmod(val, _wrapRange);
      if (std::abs(val) > (_wrapRange / 2.0)) {
        return (val > 0) ? val - _wrapRange : val + _wrapRange;
      }
    }

    return val;
  }
  double _sum = 0;
  double _previousError = 0;
};