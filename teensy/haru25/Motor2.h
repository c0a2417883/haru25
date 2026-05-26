#pragma once

#define ENCODER_PPR (2048)
#define RAD2QPPS(omega) ((omega)*ENCODER_PPR/(2*M_PI)*4)//ラジアンパーセックからカウントパーセック　半径はOmniDriveで

// ロボクローをつなぐシリアルポート

RoboClaw roboclaw_(&Serial5,0);
bool motor_init_ = false;

class Motor2 {
  int addr_;
   public:
    Motor2(int addr)
      : addr_(addr) 
      {
    }

    void init() {
      if(!motor_init_){
        roboclaw_.begin(115200);
        motor_init_ = true;
      }
      
      //Set PID Coefficients
      setSpeed(0, 0);
    }  
    void setSpeed(float omega1, float omega2) {
        roboclaw_.SpeedM1M2(addr_, (int)RAD2QPPS(omega1), (int)RAD2QPPS(omega2));
    }
};
