#pragma once
#include <cmath>

const int8_t KNOBDIR[] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};
    
int pin1a_;//プライベート変数は最後にアンダーバーをつける
int pin1b_;
volatile int counter1_;
volatile int prev_state1_;

int pin2a_;//プライベート変数は最後にアンダーバーをつける
int pin2b_;
volatile int counter2_;
volatile int prev_state2_;

void enc_count1(){
 int sig1 = digitalRead(pin1a_);
 int sig2 = digitalRead(pin1b_);
 int this_state = sig1 | (sig2 << 1);// |はオア（or）演算子 、　<<はシフト演算子
 int plus = KNOBDIR[this_state | (prev_state1_ << 2)];
 counter1_ += plus;
 prev_state1_ = this_state;
}
void enc_count2(){
 int sig1 = digitalRead(pin2a_);
 int sig2 = digitalRead(pin2b_);
 int this_state = sig1 | (sig2 << 1);
 int plus = KNOBDIR[this_state | (prev_state2_ << 2)];
 counter2_ += plus;
 prev_state2_ = this_state;
}

class Encoder{  
  public:
  Encoder(){
  }
  
  void init(){
    pin1a_ = 0;//プライベート変数は最後にアンダーバーをつける
    pin1b_ = 1;
    counter1_ = 0;

    pin2a_ = 11;//プライベート変数は最後にアンダーバーをつける
    pin2b_ = 12;
    counter2_ = 0;

    
    pinMode(pin1a_, INPUT_PULLUP);
    pinMode(pin1b_, INPUT_PULLUP);
    int sig1 = digitalRead(pin1a_);
    int sig2 = digitalRead(pin1b_);
    prev_state1_ = sig1 | (sig2 << 1);
    attachInterrupt(pin1a_, enc_count1, CHANGE); // クラス内の関数を呼べない
    attachInterrupt(pin1b_, enc_count1, CHANGE);
    
    pinMode(pin2a_, INPUT_PULLUP);
    pinMode(pin2b_, INPUT_PULLUP);
    sig1 = digitalRead(pin2a_);
    sig2 = digitalRead(pin2b_);
    prev_state2_ = sig1 | (sig2 << 1);
    attachInterrupt(pin2a_, enc_count2, CHANGE);
    attachInterrupt(pin2b_, enc_count2, CHANGE);
  }
  void getCounts(int & cnt1,int & cnt2){
    cnt1 = counter1_;
    cnt2 = counter2_;
  }
};
