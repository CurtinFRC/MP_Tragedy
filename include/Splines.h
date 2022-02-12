#pragma once

#include <math.h>
#include <iostream>
#ifdef __linux__ 
  #include <bits/stdc++.h>
#elif _WIN32
  #include <iomanip>
#endif

#include <vector>
#include <math.h>

#ifndef M_PI // Sometimes windows is wack and doesn't have M_PI
#define M_PI 3.14159265358979323846264338327
#endif

// Giving progress bar defines. (60 wide)
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60


class Splines {
 public:
  struct Waypoint {
    double x, y;
    double segLength = 0; // length of segment to next waypoint
    double totalLength = 0; // length of all segments up to and including this segment
  };

  struct SplinePoint {
    Waypoint waypoint;
    int flag;
  };

  struct Spline {
    std::vector<Waypoint> waypoints; // waypoints where the robot goes through (the first and last waypoints are ignored in this process [controlpoints])
    double totalLength = 0; // total length of spline (all segments)
    double segmentNum = 0; // number of segments in spline with a value
  };

  int buildPath(Spline &spline, int removeNodes = 0);
  double calculateSegLength(int node, Spline spline);

  void setStepSize(double step) {
    _stepSize = step;
  };

  SplinePoint getSplinePoint(float t, Spline spline);
  SplinePoint getSplineGradientPoint(float t, Spline spline);
  double getSplineAngleRad(float t, Spline spline);
  void printProgress(double percentage);
  double getSplineAngleDeg(float t, Spline spline);

 private:
  double splineLength = 0;
  double _stepSize = 0.001;
};
