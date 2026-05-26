
#pragma once

//test0
//スタート
/* Test1 OK(青いシールド基板)*/
void test1_init(){ //Air
  air_lift.init();
  air_catch.init();
}
void test1_loop(){
  air_lift.write(AIR_LIFT_UP);//false 29 0 黄色 
  air_catch.write(AIR_CATCH_ON);//false 28 青 
  delay(2000);
  air_lift.write(AIR_LIFT_DOWN);//true 29 3.0 黄色 
  air_catch.write(AIR_CATCH_OFF);//true 28 青 
  delay(2000);
}

/* Test2 OK*/ //モーターだけ動かす　
void test2_init(){//正回転か確かめる(roboclawに直接値をいれて動かす）正回転の向きは図にかいてある
  m12.init();
  m34.init();
}
void test2_loop(){
  m12.setSpeed(-1,-1);
  m34.setSpeed(-1,-1);//ロボクロー二つをsakuraにつなげる　かっこの中を-1にすると逆転するか確認 すべて１だと左回転 すべて-1だと右回転
}

/* Test3  OK*/
void test3_init(){
  m12.init();
  m34.init();
}
void test3_loop(){//オムニホイールがうまくいっているか確かめる 
  omni.setRobotSpeed(0,0.5,0,0);//一個だけ1にして試す マイナスにしても試す
  delay(1000);
}

/* Test4 OK*/
void test4_init(){//ジャイロセンサが反時計回りが正になっているか確認 
  im.init();
}
void test4_loop(){
  Serial.println(im.getYaw());
  delay(100);
}

/* Test5 */
void test5_init(){//tofで壁を読めるか確認。そのために壁と機体の中心を測ってメモすること
   tof.init();
}
void test5_loop(){
  float d1, d2, d3, d4;
  tof.getDistance(0, d1);
  tof.getDistance(1, d2);
  tof.getDistance(2, d3);
  tof.getDistance(3, d4);
  Serial.println(d1);
  Serial.print(", ");
  Serial.print(d2);
  Serial.print(", ");
  Serial.print(d3);
  Serial.print(", ");
  Serial.println(d4);
  delay(1000);
}
/* Test6 多分OK*/
void test6_init(){
   encoder.init();
}
void test6_loop(){//オドメーターの角度を表示　オドメーターを手で転がして正回転の方向を確認
      int cnt1, cnt2;
      encoder.getCounts(cnt1, cnt2);
      Serial.print(cnt1);//マイナス
      Serial.print(", ");
      Serial.println(cnt2);//マイナス
      delay(100);
}

/* Test7 OK*/
void test7_init(){//オドメーターを手で転がして、前進したらyが大きく、右に進んだらxが大きくなることを確認。このときロボットは方向を本番と合わせること。もし逆転していたらnatu24.inoの33行目のwheel_radiusをマイナスにする。
      encoder.init();
      im.init();
      odom.init();    
}
void test7_loop(){//その後ロボットを回転させたりして自己位置が正しいか確認
       Pos odom_pos = odom.get();
       
       odom_pos.print();//common.hの12行目から
       delay(10);
}
/*Test 8 OK*/
void test8_init(){//スイッチのテスト
      sw_tact1.init();
      sw_tact2.init();
      sw_tact3.init();
      sw_catch.init();
}
void test8_loop(){
      Serial.println(sw_tact1.isPushed());//ボタンを押すとLOWが返ってくる。！をつけるとHIGHになる。
      Serial.println(sw_tact2.isPushed());
      Serial.println(sw_tact3.isPushed());
      
      Serial.println(sw_catch.isPushed());
      Serial.println();
      delay(1000);
      
}
