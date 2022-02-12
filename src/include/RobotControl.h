#include <algorithm>
#include "Trajectory.h"
#include "PID.h"

class RobotControl {
 public:
  struct RobotConfig {
    double &distance;
    double &gyro;

    double maxSpeed = 0.4;
    double maxAcceleration = 0.1; // need to program this in
  };

  RobotControl(RobotConfig config, PID::PIDGains angleGains, PID::PIDGains linearGains) : _config(config), _angularPIDGains(angleGains), _linearPIDGains(linearGains) {
    _anglularPIDController = new PID::PIDController(_angularPIDGains);
    _linearPIDController = new PID::PIDController(_linearPIDGains);
  }

  /**
   * @brief 
   * 
   * @tparam SplineType (future proofs us for using other types of spline algorithms, catmull or hermite ect...)
   * @param trajectory 
   * @param dt 
   * @return std::pair<double, double> 
   */
  template<typename SplineType>
  std::pair<double, double> followTrajectory(Trajectory<SplineType> trajectory, double dt) {
    double leftPower = 0, rightPower = 0;
    if (_config.distance < trajectory.getRawTrajectory().totalLength) {
      
      // For this PID the goal is the end of the spline (the total length of it). Because thats where we want to slow down and end up
      _linearPIDController->setSetpoint(trajectory.getRawTrajectory().totalLength);
      // Calculate the PID for the linear driving
      double linearOutput = _linearPIDController->calculate(_config.distance, dt);
      

      // This is setting the goal otherwise known as the setpoint
      _anglularPIDController->setSetpoint(trajectory.getAngle(_config.distance));

      // Calculate PID and return our desired value
      double angularOutput = _anglularPIDController->calculate(_config.gyro, dt);

      // Finally set left and right with the linear power and the angular.
      leftPower = linearOutput + angularOutput;
      rightPower = linearOutput - angularOutput;
    }

    return {leftPower, rightPower};
  }

  /**
   * @brief Get the Angular P I D Controller object
   * 
   * @return PID::PIDController* 
   * Main getter for the PID controller (allows us to change wrap from outside)
   */
  PID::PIDController *getAngularPIDController() {
    return _anglularPIDController;
  }

  /**
   * @brief Get the Linear P I D Controller object
   * 
   * @return PID::PIDController* 
   */
  PID::PIDController *getLinearPIDController() {
    return _linearPIDController;
  }

 public:
  RobotConfig _config;
  PID::PIDGains _angularPIDGains;
  PID::PIDGains _linearPIDGains;
  PID::PIDController *_anglularPIDController;
  PID::PIDController *_linearPIDController;
};