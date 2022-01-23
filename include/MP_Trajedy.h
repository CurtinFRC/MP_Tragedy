#include "MP_Trajedy.h"

namespace mp_trajedy {
  class MP_Trajedy : public RobotControl {
   public:
    MP_Trajedy(Config &config);
    ~MP_Trajedy();

    Config *getConfig() {
      return &_config;
    }

   private:
    Config &_config

  };
}

//get rest ;)