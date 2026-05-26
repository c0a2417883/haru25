#pragma once

class Switch{
  int pin_;
  public:
  Switch(int pin)
  :pin_(pin){
  }
  void init(){
  pinMode(pin_,INPUT);
  }
  bool isPushed(){//boolを返すときは関数の名前がisから始まる
    return !digitalRead(pin_);//0がかえってきて、ボタンをおすと1になる
  }
};

