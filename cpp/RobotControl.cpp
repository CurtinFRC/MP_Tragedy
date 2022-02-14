#include "RobotControl.h"
#include "Splines.h"

// Splines::SplinePoint RobotControl::locationOnPath(float t, Spline spline) {
//   SplinePoint robotCoords = getSplinePoint(t, spline);

//   return robotCoords;
// }

RobotControl::FollowInfo RobotControl::followSpline(double dt) {
  std::cout << "\nDistance on spline: " << _path.getLength() << std::endl;
  FollowInfo info;

  if (_config.distance < _path.getLength()) {
    info.left = 0.15;
    info.right = 0.15;

    info.goal_angle = (_path.getAngleDeg(_config.distance) - _path.getAngleDeg(0));
    double robotAngle = _config.gyro;
    std::cout << "\nGoal Angle: " << info.goal_angle << std::endl;

    double output = _anglePID.calculate(robotAngle, info.goal_angle, dt);
 
    info.left += output;  // replace with anglePID
    info.right -= output;  // replace with anglePID

    info.is_done = false;
  } else {
    info.is_done = true;
  }
  return info;
}