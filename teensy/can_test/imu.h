#include <iterator>
#include "Adafruit_BNO055.h"
#pragma once
#include "common.h"

#define BNO_WIRE Wire

Adafruit_BNO055 bno_ = Adafruit_BNO055(OPERATION_MODE_IMUPLUS, 0x28, &BNO_WIRE);

float _imu_data[7];

void _imu_callBack(){
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Quaternion q = bno_.getQuat();
  double x = q.x();
  double y = q.y();
  double z = q.z();
  double w = q.w();
  
  double sqw = w * w;
  double sqx = x * x;
  double sqy = y * y;
  double sqz = z * z;

  _imu_data[0] = atan2(2.0 * (x * y + z * w), (sqx - sqy - sqz + sqw));//弧度法　反時計回りが正
  imu::Vector<3> acc = bno_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  _imu_data[1] = acc.x();
  _imu_data[2] = acc.y();
  _imu_data[3] = acc.z();
  _imu_data[4] = gyro.x();
  _imu_data[5] = gyro.y();
  _imu_data[6] = gyro.z();
}

class Imu {
  int reset_pin_;
  int int_pin_;
  double prev_yaw_;

  public:
  Imu(int reset_pin,int int_pin):reset_pin_(reset_pin), int_pin_(int_pin){}
  void init(){
    // リセット
    pinMode(reset_pin_, OUTPUT);
    digitalWrite(reset_pin_, LOW);
    delay(100);
    pinMode(reset_pin_, INPUT);
    delay(100);
    pinMode(int_pin_, INPUT);
    delay(100);
    
    // 通信するまでループ
    BNO_WIRE.setClock(400000); // use 400 kHz I2C 
    while(!bno_.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      delay(1000);
    }
    // 通信ができたら抜ける
    // attachInterrupt(int_pin_, _imu_callBack, RISING);
    prev_yaw_ = getYaw();
  }
  // 0,427 ms ~ 2 ms
  double getYaw(){
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
 void update(float *data){
    // _imu_callBack();
    // memcpy((uint8_t*)data, (uint8_t*)_imu_data, 7*4);
    double now_yaw = getYaw();
    // imu::Vector<3> line = bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // imu::Vector<3> gyro = bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    data[0] = normalizeAngle(now_yaw - prev_yaw_);
    prev_yaw_ = now_yaw;
    // data[1] = line.x();
    // data[2] = line.y();
    // data[3] = line.z();
    // data[4] = gyro.x();
    // data[5] = gyro.y();
    // data[6] = gyro.z();
    // data[7] = acc.x();
    // data[8] = acc.y();
    // data[9] = acc.z();
    // data[10] = grav.x();
    // data[11] = grav.y();
    // data[12] = grav.z();
  }
};
