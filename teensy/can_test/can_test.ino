#include <QuadEncoder.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <PacketSerial.h>  //PCと通信する用　送り方
#include <FlexCAN_T4.h>
#include <WS2812Serial.h>


#include "pi.h"
#include "robomas.h"
#include "packet.h"
#include "imu.h"
#include "encoder.h"
#include "air.h"
#include "tapeled.h"

// Imu im(32,26); 
Imu im(20,17); //るなちゃんの基板　Wireを使用
// Imu im(26,27); 
// Imu im(27,26);//よしまさの基板　Wire2を使用
Encoder encoder;
// Air air(40,39,38,37); //よしまさの基板
Air air(29,32,27,28); //るなちゃんの基板
Tapeled tapeled;

int pre_cnt1 = 0;
int pre_cnt2 = 0;
float air_tmp = 0;
int pre_air_data = 0;
float tape_led = 0;
int pre_tape_led = 0;
unsigned long prev_connect_ros2_ts_ = 0;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can1;  //teensyのcan1を登録　RX_SIZE_256→受信buffer データをためとく　TX_SIZE_16→送信buffer
CAN_message_t msg, msg2;
Robomas robomas[4];
bool update_odom = false;
float odom[9];//x, y, yaw + acc3 + gyro3 + lin3 + grav3

// PCから受信時に一度呼ばれるやつ
void packet_FrameCallBack() {
  prev_connect_ros2_ts_ = millis();
  // // PCに送信するデータを登録
  for (int i = 0; i < 4; ++i) {
    float f[6] = { robomas[i].input_curr, robomas[i].state_curr, robomas[i].input_vel, robomas[i].state_vel, robomas[i].input_pos, robomas[i].state_pos };
    packet_setData(i, f, 6);
  }
  // オドメトリが更新されたら
  packet_setData(0xff, odom, 3);
}

// // PCから受信時にパケットごとに呼ばれるやつ
void packet_PacketCallBack(const uint8_t id, const float *data, const size_t len) {
  if (id < 4) {
    robomas[id].setData(data, len);
  } else if (id == 4 && len == 1) {
    air_tmp = data[0];
  } else if (id == 5 && len == 1) {
    tape_led = data[0];
  }
}

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  robomas[0].m2006 = false;
  robomas[1].m2006 = false;
  robomas[2].m2006 = false;
  robomas[3].m2006 = false;
  im.init();
  encoder.init();
  encoder.getCounts(pre_cnt1, pre_cnt2);
  air.init();
  tapeled.init();
  can1.begin();
  can1.setBaudRate(1000000);  //canの通信速度設定
  packet_begin();
}

unsigned long prev_ts = 0;
bool led = false;
int led_count = 0;
void loop() {
  // オドメトリ 200Hz
    static unsigned long prev_imu_ts = 0;
    static bool imu_update = false;
    unsigned long now_ts = micros();
  // 200Hz
  if(now_ts - prev_imu_ts >= 5000){
    prev_imu_ts = now_ts;

    int cnt1, cnt2;
    encoder.getCounts(cnt1, cnt2);
    float diff1, diff2;
    diff1 = (float)(cnt1 - pre_cnt1);
    diff2 = (float)(cnt2 - pre_cnt2);
    pre_cnt1 = cnt1;
    pre_cnt2 = cnt2;
    odom[0] = diff1;
    odom[1] = diff2;

    imu_update = !imu_update;
    if(imu_update){
      im.update(odom+2);
    }
  }
  // PCとの通信 200Hz
  packet_update();

  // C620との通信 1kHz
  if (can1.read(msg)) {  //c610から受信があったら入る
    // Serial.println("ok");//動かすときはここをコメントアウトにしないとエラー
    if ((msg.id >= (0x200 + 1)) && msg.id <= (0x200 + 4) && msg.len == 8) {  //現在の測定値をクラスにいれてる
      int robomas_i = msg.id - 0x200 - 1;
      robomas[robomas_i].setState(msg.buf);  //robomas.state_pos,robomas.state_vel,robomas.state_currを計算
    }
  }
  if (now_ts - prev_ts >= 1000) {  //1khzで入る　ここでロボマス動く
    prev_ts = now_ts;
    msg2.id = 0x200;  //canid
    msg2.len = 8;     //フレーム(canのデータのかたまり)のサイズ
    int16_t ref[4];   //8byteのデータにしてる
    for (int i = 0; i < 4; ++i)
      ref[i] = robomas[i].getRefCurrent();  //ロボマスの目標電流を計算
    uint8_t *ref8 = (uint8_t *)ref;         //見ていくかたまりを1byteづつにした
    for (int i = 0; i < 4; ++i) {
      msg2.buf[i * 2 + 1] = ref8[i * 2];
      msg2.buf[i * 2] = ref8[i * 2 + 1];
    }
    can1.write(msg2);
    // 20Hz
    if(++led_count == 50){
      led_count = 0;
      if(millis() - prev_connect_ros2_ts_ > 200){
        // ros2と1秒以上接続してない
        led = true;
      }else{
        // 接続中
        led = !led;
      }
      digitalWrite(LED_BUILTIN, led);
    }
  }
  // エア
  int air_data = (int)air_tmp;
  if (air_data != pre_air_data) {
    pre_air_data = air_data;
    air.write(bitRead(air_data, 0), bitRead(air_data, 1), bitRead(air_data, 2), bitRead(air_data, 3));  //0000 が 0001 になるときは０桁目のAirがonになってる
  }
  // テープLED
  int tapeled_data = (int)tape_led;  //floatからintに変換
  if (tapeled_data != pre_tape_led) {
    int microsec = 1500000 / leds.numPixels();
    pre_tape_led = tapeled_data;
    tapeled.colorWipe(tapeled_data, microsec);
  }
}