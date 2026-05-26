#pragma once
#include <cmath>

QuadEncoder myEnc1(1, 3, 2, 0);  // Encoder on channel 1 of 4 available
                                 // Phase A (pin0), PhaseB(pin1), Pullups Req(0)
QuadEncoder myEnc2(2, 4, 5, 0);  // Encoder on channel 2 of 4 available
                                 //Phase A (pin2), PhaseB(pin3), Pullups Req(0)
//るなちゃん　3,2と4,5

// QuadEncoder myEnc1(1, 4, 8, 0); 
// QuadEncoder myEnc2(2, 5, 7, 0);
// //よしまさ　4,8と5,7


class Encoder{  
  public:
  Encoder(){
  }
  
  void init(){
    while(!Serial && millis() < 4000);

  /* Initialize the ENC module. */
   myEnc1.setInitConfig();  
   myEnc1.EncConfig.revolutionCountCondition = ENABLE;
   myEnc1.EncConfig.enableModuloCountMode = DISABLE;
   myEnc1.EncConfig.positionModulusValue = 20; 
   myEnc1.init();
  
   myEnc2.setInitConfig();  
   myEnc2.EncConfig.revolutionCountCondition = ENABLE;
   myEnc2.EncConfig.enableModuloCountMode = DISABLE;
   myEnc2.EncConfig.positionModulusValue = 20; 
   myEnc2.init();

  }
  void getCounts(int & cnt1,int & cnt2){
     cnt1 = myEnc1.read();
     cnt2 = myEnc2.read();
  }
};
