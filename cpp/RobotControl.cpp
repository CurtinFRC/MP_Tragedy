#include "RobotControl.h"
#include "Splines.h"

double RobotControl::dist2t(double distance, Splines::Spline spline) {
  int segment = 0;

  std::cout << "Break before loop" << std::endl;

  for (int i = 0; i < spline.waypoints.size(); i++) {
    if (distance > spline.waypoints[segment].totalLength) {
      segment++;
      break;
    }
  }

  std::cout << "Break after loop" << std::endl;

  std::cout << "Segment: " << segment << std::endl;

  double distanceAlongSegment = 0;
  if (segment != 0) {
    distanceAlongSegment = distance - spline.waypoints[segment-1].totalLength;
  } else {
    distanceAlongSegment = distance;
  }

  // scaledNum = ((x-a)*(d-c)/(b-a))+c
  double oldRange = (spline.waypoints[segment].segLength - 0);
  double newRange = (1-0);

  double t = (((distanceAlongSegment - 0) * newRange) / oldRange) + 0;

  return t+segment;
}

float RobotControl::tValue(double encoderRotations, double splineLength) {
  double meters = encoderRotations / splineLength;
  float t = meters / 16;
  
  std::cout << "\nrotations: " << encoderRotations << std::endl;
  std::cout << "meters: " << meters << std::endl;
  std::cout << "t: " << t << std::endl;

  return t;
}

Splines::SplinePoint RobotControl::locationOnPath(float t, Splines::Spline spline) {
  Splines::SplinePoint robotCoords = Splines::getSplinePoint(t, spline);

  return robotCoords;
}

double RobotControl::followSpline(float t, Splines::Spline spline) {
  double motorSpeeds [2];

  // SplinePoint gradient = CatmullRom::getSplineGradientPoint(t, spline);


  double angle = Splines::getSplineAngleDeg(t, spline);
   
  // std::cout << "angle x: " << angle.waypoint.x << std::endl;
  std::cout << "Goal Angle: " << angle << std::endl;
  // std::cout << "Distance along segment: " << dist2t()
  
  return 0;
}