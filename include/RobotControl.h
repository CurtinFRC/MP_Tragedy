#pragma once
#include <math.h>
#include <iostream>
#ifdef __linux__ 
  #include <bits/stdc++.h>
#elif _WIN32
  #include <iomanip>
#endif

#include "PID.h"
#include "Trajectory.h"


class RobotControl {
 public:
  struct Config {
    double &distance;  // avarage distance between two encoders
    double &gyro;
  };

  RobotControl(Trajectory path, Config config, PID anglePID) : _path(path), _config(config), _anglePID(anglePID) {
    _anglePID.setWrap(180);
  }

  Splines::SplinePoint locationOnPath();
  std::pair<double, double> followSpline(double dt);


  Config getConfig() {
    return _config;
  }

  PID &getAnglePID() {
    return _anglePID;
  }

 private:
  Trajectory _path;
  Config _config;
  float t = 0.0;
  double totalRotations = 0;

  // PID _linearPID;
  PID _anglePID;
  
};