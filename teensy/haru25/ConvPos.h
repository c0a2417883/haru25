#pragma once
#include "Common.h"

class ConvPos{
  float param_[7];
  public:
  ConvPos(){}
  // 座標を設定する
  void init(Pos &odom_pos, Pos &init_pos){
    // 座標設定時のオドメトリ
    param_[0] = odom_pos.x;
    param_[1] = odom_pos.y;
    // 設定座標
    param_[2] = init_pos.x;
    param_[3] = init_pos.y;
    // 角度変換用
    param_[4] = init_pos.t - odom_pos.t;
    param_[5] = cosf(param_[4]);
    param_[6] = sinf(param_[4]);
  }
  // 座標を変換
  Pos get(Pos &odom_pos){
    float dx = odom_pos.x - param_[0];
    float dy = odom_pos.y - param_[1];
    Pos global_pos;
    global_pos.x = (dx * param_[5] - dy * param_[6]) + param_[2];
    global_pos.y = (dx * param_[6] + dy * param_[5]) + param_[3];
    global_pos.t = normalizeAngle(odom_pos.t + param_[4]);
    return global_pos;
  } 
};

