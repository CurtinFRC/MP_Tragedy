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
  struct FollowInfo {
    bool is_done;
    double left, right;
    double goal_angle;
  };

  RobotControl(Trajectory path, PID anglePID) : _path(path), _anglePID(anglePID) {
    _anglePID.setWrap(180);
  }

  Splines::SplinePoint locationOnPath();
  FollowInfo followSpline(double dt, double distance, double gyro);

  PID &getAnglePID() {
    return _anglePID;
  }

 private:
  Trajectory _path;
  float t = 0.0;
  double totalRotations = 0;

  double _maxSpeed = 0;

  // PID _linearPID;
  PID _anglePID;
  
};