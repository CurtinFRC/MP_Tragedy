#include "Trajectory.h"
#include "PID.h"

class RobotControl {
 public:
  struct RobotConfig {
    double &distance;
    double &gyro;
  };

  RobotControl(RobotConfig config, PID::PIDGains gains) : _config(config), _pidGains(gains) {
    _pidController = new PID::PIDController(_pidGains);
  }

  template<typename SplineType>
  std::pair<double, double> followTrajectory(Trajectory<SplineType> trajectory) {
    if (!trajectory.atWaypoint(trajectory.getRawTrajectory().points.size()-2))
  }

  PID::PIDController *getPIDController() {
    return _pidController;
  }

 public:
  RobotConfig _config;
  PID::PIDGains _pidGains;
  PID::PIDController *_pidController;
};