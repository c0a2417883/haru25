#pragma once

class Air{
  int pin1_;
  int pin2_;
  int pin3_;
  int pin4_;
  public:
  Air(int pin1,int pin2, int pin3, int pin4)
   :pin1_(pin1),
   pin2_(pin2),
   pin3_(pin3),
   pin4_(pin4){
  }
  void init(){
    pinMode(pin1_,OUTPUT);
    pinMode(pin2_,OUTPUT);
    pinMode(pin3_,OUTPUT);
    pinMode(pin4_,OUTPUT);
  }

  void write(bool flag1, bool flag2, bool flag3, bool flag4){
    digitalWrite(pin1_,flag1);
    digitalWrite(pin2_,flag2);
    digitalWrite(pin3_,flag3);
    digitalWrite(pin4_,flag4);
  }
};
