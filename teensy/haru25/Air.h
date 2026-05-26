#pragma once

class Air{
  int pin_;
  int pin_led_;
  public:
  Air(int pin,int pin_led)
   :pin_(pin),
    pin_led_(pin_led){
  }
  void init(){
    pinMode(pin_,OUTPUT);
    pinMode(pin_led_,OUTPUT);
  }
  void expand(){
    write(true);
  }
  void shrink(){
    write(false);
  }
  void write(bool flag){
    digitalWrite(pin_,flag);
    digitalWrite(pin_led_,flag);
  }
};
