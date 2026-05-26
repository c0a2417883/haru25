#pragma once
#include "Common.h"

Adafruit_BNO055 bno_ = Adafruit_BNO055(OPERATION_MODE_NDOF, 0x28, &SENSOR_WIRE_GYRO);

class Imu {
  int reset_pin_;
  public:
  Imu(int reset_pin):reset_pin_(reset_pin){}
  void init(){
    // リセット
    pinMode(reset_pin_, OUTPUT);
    digitalWrite(reset_pin_, LOW);
    delay(100);
    pinMode(reset_pin_, INPUT);
    delay(100);
    
    // 通信するまでループ
    SENSOR_WIRE_GYRO.setFrequency(400000); // use 400 kHz I2C 
    while(!bno_.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      delay(1000);
    }
    // 通信ができたら抜ける
  }
  // 0,427 ms ~ 2 ms
  float getYaw(){
    // Quaternionで読み取った方が精度が良い
    //secondary.blog.fc2.com/blog-entry-50.html
    // Quaternionを読み取り
    imu::Quaternion q = bno_.getQuat();
    double x = q.x();
    double y = q.y();
    double z = q.z();
    double w = q.w();
    
    double sqw = w * w;
    double sqx = x * x;
    double sqy = y * y;
    double sqz = z * z;

    return atan2(2.0 * (x * y + z * w), (sqx - sqy - sqz + sqw));//弧度法　反時計回りが正

//      sensors_event_t orientationData;
//      bno_.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//      return DEG2RAD(orientationData.orientation.x);
  }
};
