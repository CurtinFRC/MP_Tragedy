#include "Trajectory.h"

Trajectory::Trajectory(std::vector<Trajectory::Waypoint> points) : _trajectory{points} {  // makes spline with points
}

void Trajectory::appendWaypoint(Trajectory::Waypoint point) {
  _trajectory.waypoints.push_back(point);
}

void Trajectory::build() {
  buildPath(_trajectory, 0);
}

double Trajectory::getLength() {
  return _trajectory.totalLength;
}

float Trajectory::dist2t(double distance) {
  int segment = 0;

  for (int i = 0; i < _trajectory.waypoints.size(); i++) {
    if (distance > _trajectory.waypoints[segment].totalLength) {
      segment++;
      break;
    }
  }

  double distanceAlongSegment = 0;
  if (segment != 0) {
    distanceAlongSegment = distance - _trajectory.waypoints[segment-1].totalLength;
  } else {
    distanceAlongSegment = distance;
  }

  // scaledNum = ((x-a)*(d-c)/(b-a))+c
  double oldRange = (_trajectory.waypoints[segment].segLength - 0);
  double newRange = (1-0);

  double t = (((distanceAlongSegment - 0) * newRange) / oldRange) + 0;

  return t+segment;
}

double Trajectory::getAngleDeg(double distance) {
  float t = dist2t(distance);
  return getSplineAngleDeg(t, _trajectory);
}

void atWaypoint(int node) {
  // get spline point
}
