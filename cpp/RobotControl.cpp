#include "RobotControl.h"
#include "Splines.h"

// Splines::SplinePoint RobotControl::locationOnPath(float t, Spline spline) {
//   SplinePoint robotCoords = getSplinePoint(t, spline);

//   return robotCoords;
// }

std::pair<double, double> RobotControl::followSpline(double dt) {
  std::cout << "\nDistance on spline: " << _path.getLength() << std::endl;
  double leftPower = 0, rightPower = 0;

  if (_config.distance < _path.getLength()) {
    leftPower = 0.25;
    rightPower = 0.25;



    double goalAngle = _path.getAngleDeg(_config.distance);
    double robotAngle = _config.gyro;
    std::cout << "\nGoal Angle: " << goalAngle << std::endl;

    // double output = _anglePID.calculate(robotAngle, goalAngle, dt);
 
    // leftPower += output;  // replace with anglePID
    // rightPower -= output;  // replace with anglePID
  }
  return {leftPower, rightPower};
}