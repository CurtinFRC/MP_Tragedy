#pragma once
#include "Splines.h"
#include "PID.h"

class Trajectory : public Splines {
 public: 
  Trajectory() {}
  Trajectory(std::vector<Waypoint> points);

  void appendWaypoint(Waypoint point);
  void appendWaypoint(std::vector<Waypoint> points) {
    for (auto point : points) {
      appendWaypoint(point);
    }
  }

  // double getTrajectoryAngle(double distance) {
  //   double t = dist2t(distance, _trajectory);
  //   return getSplineAngleDeg(t, _trajectory);
  // }

  double getLength();
  void build();

  // float tValue(RobotControl::distance);

  bool atWaypoint(int node); // node = waypoint

  float dist2t(double distance);
  double getAngleDeg(double distance);

  Spline &getRawTrajectory() {
    return _trajectory;
  }
 private:
  Spline _trajectory;
};