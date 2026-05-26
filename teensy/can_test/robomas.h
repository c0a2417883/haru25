#pragma once

#include <stdint.h>
#include "pi.h"

#define ROBOMAS_CONTROL_DT 0.001f
#define ROBOMAS_CONNECT_TIMEOUT 1000 //ms

class Robomas{
    float rx_data[4];
    int rx_len = 0;
    float prev_pos = 0.;
    int rotate = 0;
    PID pos_p;
    PID vel_pi;
    unsigned long prev_connect_can_ts = 0;
    unsigned long prev_connect_ros2_ts = 0;

  public:
     bool m2006 = true;
     float state_pos = 0.;
     float state_vel = 0.;
     float state_curr = 0.;
     float input_pos = 0.;
     float input_vel = 0.;
     float input_curr = 0.;
    Robomas(): pos_p(20., 0, ROBOMAS_CONTROL_DT), vel_pi(2, 10, ROBOMAS_CONTROL_DT){
    }
    int getRefCurrent(){
      int ref_current;
        if(rx_len == 0){//停止             rx_data = {ID} 
          input_pos = 0;
          input_vel = 0;
          input_curr = 0;
        }else if(rx_len == 1){//電流制御   rx_data = {ID, 目標電流}→rx_data = {目標電流}に変更
          input_pos = 0;
          input_vel = 0;
          input_curr = rx_data[0];//1
        }else if(rx_len == 2){//速度制御   rx_data = {ID, 目標電流, 目標速度}→rx_data = {目標電流, 目標速度}に変更
          input_pos = 0;
          input_vel = rx_data[1];//2
          input_curr = vel_pi.update(input_vel - state_vel, rx_data[0]);//1
        }else if(rx_len == 3){//位置制御   rx_data = {ID, 目標電流, 目標速度, 目標角度}→rx_data = {目標電流, 目標速度, 目標角度}に変更
          input_pos = rx_data[2];//3
          input_vel = pos_p.update(input_pos - state_pos, rx_data[1]);//2
          input_curr = vel_pi.update(input_vel - state_vel, rx_data[0]);//1
        }else if(rx_len == 4){
          vel_pi.setGain(rx_data[0], rx_data[1]);
          pos_p.setGain(rx_data[2], rx_data[3]);
          input_curr = 0;
        }
          // C620と１秒以上つながってないときは積分をリセットして、入力電流を0にする
        unsigned long now_ts = millis();
        if((now_ts - prev_connect_can_ts >= ROBOMAS_CONNECT_TIMEOUT) || (now_ts - prev_connect_ros2_ts >= ROBOMAS_CONNECT_TIMEOUT)){
          input_curr = 0;
          pos_p.reset();
          vel_pi.reset();
        }
        if(m2006){
          input_curr  = constrain(input_curr,-10.,10.);
          ref_current = input_curr * 10000. / 10.;
        }else{
          input_curr  = constrain(input_curr,-20.,20.);
          ref_current = input_curr * 16384. / 20.;
        }
            
        return ref_current;
    }
    // ロボマスから来たモーターの情報を取得
    void setState(uint8_t * buf){
      int16_t pos = (buf[0] << 8 | buf[1]);
      int16_t vel = (buf[2] << 8 | buf[3]);
      int16_t curr = (buf[4] << 8 | buf[5]);
      if( pos - prev_pos > 4096)
        rotate--;
      else if ( pos - prev_pos < -4096)
        rotate++;
      prev_pos =  pos;
      if(m2006){
        state_pos = ((float) pos / 8192 + rotate)*PI*2 / 36;
        state_vel = (float)vel / 60 * PI * 2 / 36;
        state_curr = (float)curr / 10000. * 10.;
      }else{
        state_pos = ((float) pos / 8192 + rotate)*PI*2 / 19;
        state_vel = (float)vel / 60 * PI * 2 / 19;
        state_curr = (float)curr / 16384. * 20.;
      }
      // timeout監視
      prev_connect_can_ts = millis();
    }
    void setData(const float *f, const int len){
      int size = (len >= 4) ? 4 : len;

      for(int i =0;i<size;++i){
        rx_data[i] = f[i];
      }
      // モードが切り替わったとき
      if(rx_len != size){
        pos_p.reset();
        vel_pi.reset();
      }
      rx_len = size;
      prev_connect_ros2_ts = millis();
    }
};
