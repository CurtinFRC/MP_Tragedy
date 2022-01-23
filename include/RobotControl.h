namespace mp_trajedy {
  class RobotControl {
    public:
      struct Config {
        wml::Drivetrain *drivetrain;
        bool invertLeftENC = false;
        bool invertRightENC = false;

        double *kpDrive,
        *kiDrive,
        *kpDrive,

        *kpTurn,
        *kiTurn,
        *kpTurn,

        gearBoxReduction = 0,
        wheelDiameter = 0,
        maxSpeed = 0,
        maxTurnSpeed = 0;
      };
  }
}