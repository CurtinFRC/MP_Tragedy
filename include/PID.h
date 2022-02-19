#pragma once
#include <iostream>

class PID {
 public:
  struct Gains {
    double kP,kI,kD;
    void scheduleGains(double p, double i, double d){
      kP = p;
      kI = i;
      kD = d;
    }
  };

  PID(Gains gains, double isDoneThresh) : _gains(gains), _isDoneThresh(isDoneThresh) {
    for (int i = 0; i < 20; i++) {
      _previousDerivs[i] = 0;
    }
  } // P=0.1-0.3, I=0, D=0

  void setWrap(double range) {
    _wrapRange = range;
  }

  Gains &getGains() {
    return _gains;
  }

  bool isDone() {
    double _avgDeriv = 0;
    for (int i = 0; i < 20; i++) {
      _avgDeriv += 1.0 / 20.0 * _previousDerivs[i];
    }
    return _iterations > 20 && std::abs(_avgDeriv) < _isDoneThresh;
  }

  void setIsDoneThresh(double thresh) {
    _isDoneThresh = thresh;
  }

  void setIZone(double thresh) {
    _izone = thresh;
  }
  
  double calculate(double input, double goal, double dt) {
    double error = wrap(goal - input);
    double derror = (error - _previousError) / dt;
    _sum += error * dt;

    if (error > _izone) {
      _sum = 0;
    }
    
    double output = _gains.kP * error + _gains.kI * _sum + _gains.kD * derror;

    _previousError = error;
    _previousDerivs[_iterations % 20] = derror;
    _iterations++;

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

  int _iterations = 0;
  double _previousDerivs[20];
  double _sum = 0;
  double _previousError = 0;
  double _isDoneThresh = 0;
  double _izone = std::numeric_limits<double>::max();
};