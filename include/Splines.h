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

  static int buildPath(Spline &spline, int removeNodes = 0);
  static double calculateSegLength(int node, Spline spline);

  static void setStepSize(double step) {
    _stepSize = step;
  };

  static SplinePoint getSplinePoint(float t, Spline spline);
  static SplinePoint getSplineGradientPoint(float t, Spline spline);
  static double getSplineAngleRad(float t, Spline spline);
  static void printProgress(double percentage);
  static double getSplineAngleDeg(double t, Spline spline);

 private:
  static double splineLength;
  static double _stepSize;
};