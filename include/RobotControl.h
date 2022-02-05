#include <math.h>
#include <iostream>
#ifdef __linux__ 
  #include <bits/stdc++.h>
#elif _WIN32
  #include <iomanip>
#endif

#include "Splines.h"


class RobotControl {
 public:
  static float tValue(double encoderRotations, double splineLength);
  static Splines::SplinePoint locationOnPath(float t, Splines::Spline spline);
  static double followSpline(float t, Splines::Spline spline);
  static double dist2t(double distance, Splines::Spline spline);

 private:
  float t = 0.0;
  double totalRotations = 0;
};