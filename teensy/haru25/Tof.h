#pragma once

const uint8_t sensorCount = 4;//センサの個数
const uint8_t xshutPins[sensorCount] = {33,35,30,31};//33,35,A2,A3で　一つずつやる
VL53L1X sensors[sensorCount];

class Tof{
  public:
  Tof(){}
  void init(){
    SENSOR_WIRE.begin();
    SENSOR_WIRE.setFrequency(400000); // use 400 kHz I2C
    
    for (uint8_t i = 0; i < sensorCount; i++) {
      pinMode(xshutPins[i], OUTPUT);
      digitalWrite(xshutPins[i], LOW);
      sensors[i].setBus(&SENSOR_WIRE);
    }
    for (uint8_t i = 0; i < sensorCount; i++) {
      digitalWrite(xshutPins[i], HIGH);
      delay(10);
      sensors[i].setTimeout(250);
      if (!sensors[i].init()) {
        Serial.print("Failed to detect and initialize sensor ");
        Serial.println(i);
        while (1);
      }
      sensors[i].setAddress(0x2A + i);
      sensors[i].setDistanceMode(VL53L1X::Short);
      sensors[i].setMeasurementTimingBudget(100000); //測定タイミングバジェット(1回の距離測定)に許容される時間[micros]
      sensors[i].startContinuous(50); //連続測定モードの測定間隔[msec]　0を指定すると可能な限り最速になる
    }
  }
  bool getDistance(int i, float &distance){
    distance = sensors[i].read(true);
    return !sensors[i].timeoutOccurred();
  }
};

