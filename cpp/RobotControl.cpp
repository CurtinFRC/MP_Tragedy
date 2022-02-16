#include "RobotControl.h"
#include "Splines.h"
#include "math.h"

// Splines::SplinePoint RobotControl::locationOnPath(float t, Spline spline) {
//   SplinePoint robotCoords = getSplinePoint(t, spline);

//   return robotCoords;
// }

RobotControl::FollowInfo RobotControl::followSpline(double dt, double distance, double gyro) {
  std::cout << "\nDistance on spline: " << _path.getLength() << std::endl;
  FollowInfo info;

  _accSpeed += 0.3 * dt;

  // _maxSpeed = std::min(_maxSpeed + 0.3 * dt, 0.3);

  // if (distance < _path.getLength()) {
  double distPID = _distancePID.calculate(distance, _path.getLength(), dt);
  double speed = std::min(0.3, std::min(_accSpeed, distPID));
  info.left = speed;
  info.right = speed;

  if (distance < _path.getLength())
    info.goal_angle = (_path.getAngleDeg(distance) - _path.getAngleDeg(0));
  else 
    info.goal_angle = (_path.getAngleDeg(_path.getLength() - 0.01) - _path.getAngleDeg(0));
  
  double robotAngle = gyro;
  std::cout << "\nGoal Angle: " << info.goal_angle << std::endl;

  double output = _anglePID.calculate(robotAngle, info.goal_angle, dt);

  info.left += output;  // replace with anglePID
  info.right -= output;  // replace with anglePID

  // TODO: Come back to this
  info.is_done = false;
  //   info.is_done = false;
  // } else {
  //   info.is_done = true;
  // }
  return info;
}