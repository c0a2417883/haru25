#pragma once
#include "Common.h"
#include "Tof.h"

class CorrectPos {
    Tof *tof_;
    float tof_pos_[4][2] = {{-0.22, 0.22}, {0.22, 0.22}, {-0.22, 0.22}, {0.22, 0.22}};  // tofの位置　(x, y)×４
    float tof_angle_[2] = {0, -M_PI_2};                                                 // tofの向き
   public:
    CorrectPos(Tof *tof)
        : tof_(tof) {
    }
    // 壁までの距離と、壁の角度を返す
    Wall getWall(bool horizontal_tof) {
        Wall wall;
        float x1, x2, y1, y2;
        float d1, d2;
        int tof1, tof2;
        // tofを選択
        if (horizontal_tof) {
          tof1 = 0;
          tof2 = 1;
        }else{
          tof1 = 2;
          tof2 = 3;
        }
        // 読み取り失敗
        if(!(tof_->getDistance(tof1, d1)&&tof_->getDistance(tof2, d2))){
          wall.d = -1;
          return wall;
        }
        
        // ロボットの中心を原点にする
        x1 = tof_pos_[tof1][0];
        y1 = tof_pos_[tof1][1] + d1;
        x2 = tof_pos_[tof2][0];
        y2 = tof_pos_[tof2][1] + d2;
        
        Serial.print("\nTof: ");
        Serial.print(x1);
        Serial.print(", ");
        Serial.print(y1);
        Serial.print(", ");
        Serial.print(x2);
        Serial.print(", ");
        Serial.println(y2);
        float a = y2 - y1;
        float b = x2 - x1;
        float c = x1*y2-y1*x2;
        wall.d = fabs(c) / hypotf(a, b);  // 点と直線の距離
        wall.t = atan2f(a, b);
        return wall;
    }
    // 位置を修正。見たい壁にtofが向いていることを想定
    // 現在の位置、壁のxかy座標、垂直な壁か
    Pos collectPos(Pos &pos, float wall_xy, bool wall_vertical) {
        // 使うtofを見つける
        // ロボットから見た壁の角度
        float wall_angle;
        // 距離の符号
        float d_sign;
        if (wall_vertical) {
            if (wall_xy > pos.x) {
                wall_angle = 0;
                d_sign = -1;
            } else {
                wall_angle = M_PI;
                d_sign = 1;
            }
        } else {
            if (wall_xy > pos.y) {
                wall_angle = M_PI_2;
                d_sign = -1;
            } else {
                wall_angle = -M_PI_2;
                d_sign = 1;
            }
        }
        // 壁の角度 - tofの角度
        float tof1_angle = normalizeAngle(wall_angle - (tof_angle_[0] + pos.t));
        float tof2_angle = normalizeAngle(wall_angle - (tof_angle_[1] + pos.t));

        // tof1を使うか。壁の角度と、tofの角度の差が小さい方を採用する
        int use_tof1 = fabs(tof1_angle) < fabs(tof2_angle);
        Serial.print("\nuse_tof: ");
        Serial.println(use_tof1);

        // 壁との距離と角度
        Wall wall = getWall(use_tof1);
        
        // 座標を修正
        Pos collect_pos;
        // tofの読み取りが失敗
        if(wall.d == -1){
          collect_pos.x = pos.x;
          collect_pos.y = pos.y;
          collect_pos.t = pos.t;
          Serial.println("\n Tof Time out");
          return collect_pos;
        }
        Serial.print("\n Wall: ");
        Serial.print(wall.d);
        Serial.print(", ");
        Serial.println(wall.t);
 
        if (wall_vertical) {  // Xを修正
            collect_pos.x = wall_xy + d_sign * wall.d;
            collect_pos.y = pos.y;
        } else {  // Yを修正
            collect_pos.x = pos.x;
            collect_pos.y = wall_xy + d_sign * wall.d;
        }
        // ヨー角を修正
        collect_pos.t = normalizeAngle(-wall.t + wall_angle - tof_angle_[!use_tof1]);
        return collect_pos;
    }
};

