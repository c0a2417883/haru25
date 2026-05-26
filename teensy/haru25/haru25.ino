#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <PacketSerial.h>//PCと通信する用　送り方

#include "RoboClaw.h"

#define SENSOR_WIRE (Wire2)       // ToF
#define SENSOR_WIRE_GYRO (Wire2)  // ジャイロ

//#define USE_PATH_FORWARD//右向きで受け取るときはコメントアウトする

#ifdef USE_PATH_FORWARD
#include "PathF.h"
#else
#include "Path.h"
#endif

// エンコーダー　amt103 分解能2048
#include "Air.h"
#include "ConvPos.h"
#include "CorrectPos.h"
#include "Encoder.h"
#include "Imu.h"
#include "Motor2.h"
#include "Odometry.h"
#include "OmniDrive.h"
#include "PID.h"
#include "Plan.h"
#include "PurePursuit.h"
#include "Switch.h"
#include "Tof.h"

PacketSerial myPacketSerial;

// 重要パラメーター
#define VELOCITY_LIMIT_LINEAR 0.75
#define VELOCITY_LIMIT_ANGLAR 3.14
// #define USE_TOF

// 足回り
Motor2 m12(128);  // ロボクローのアドレス
Motor2 m34(129); //クラス名　インスタンス名
OmniDrive omni(0.198, 0.05114, &m12, &m34);

// 自己位置推定
Encoder encoder;
Imu im(27);  // imuがbno055のライブラリと被ってて使えない
Odometry odom(&encoder, &im, -0.024, -0.024);
Tof tof;
ConvPos conv;
CorrectPos correct(&tof);

// 経路追従
Plan plan;
PID pid_yaw(2.0, 0.001, 0, VELOCITY_LIMIT_ANGLAR);  // 角度がフラフラするとき角度のPDゲインを上げる。Pゲインを上げてみてDゲインは発振したら上げる。
PID pid_foot(5.0, 0., 1.0, VELOCITY_LIMIT_LINEAR);  // 角度PIDが目標角度までいかないとき、PとIゲインを上げる
// path_aim_distance, path_goal_distance, path_goal_angle, path_pid_distance
PurePursuit pure_pursuit(&plan, &omni, &pid_yaw, &pid_foot, 0.1, 0.05, DEG2RAD(10.), 0.1, VELOCITY_LIMIT_LINEAR);

// 機構制御
Air air_lift(29, 51);   // 黄色 29が真ん中
Air air_catch(28, 50);  // 青色に光る  28が左はし(リセットボタンが右下)
Switch sw_catch(23);    // リミットスイッチ
Switch sw_tact1(41);    // 白41
Switch sw_tact2(42);    // 青42
Switch sw_tact3(43);    // オレンジ43

// エア関係
#define AIR_LIFT_UP (false)  // true か falseか機械班の取付で変える trueのときledが光る
#define AIR_LIFT_DOWN (true)
#define AIR_CATCH_ON (false)
#define AIR_CATCH_OFF (true)

#define AIR_CATCH_DELAY 4000      // よん秒 シェフロボがフライパンを配膳に置くとき
#define AIR_LIFT_DOWN_DELAY 2000  // テーブルゾーンに着いてリフトをおろした後
#define AIR_RELEASE_DELAY 1000    // アームを展開した後
#define BACK_LAST_DELAY 750      // 最後の経路でフライパンをおいてy座標をちょっと大きくした後（ロボットが後ろに下がった後）

// 状態推移
typedef enum {
    WAIT_START,          // ボタンが押されるのを待っている
    MOVE_START_2_CATCH,  // 受け渡し場所へ行く途中
    WAIT_CATCH,          // 受け渡し場所で待つ
    MOVE_CATCH_2_TABLE,  // テーブルゾーンへ移動中
    MOVE_TABLE_2_CATCH,  // 受け渡しゾーンへ移動中
    END                  // 全てを配達完了
} State;

// ロボットの状態
State state = WAIT_START;
// 目標のテーブルゾーン
int table_zone = 0;

// ロボットの初期位置
#ifdef USE_PATH_FORWARD
Pos init_pos = {3.218, 0.24, 0};
#else
Pos init_pos = {2.983, 0.225, -M_PI_2};
#endif

// リフトを上下動するy座標
#define LIFT_Y 1.0

// リミットスイッチの状態によって強制的にスタート
#define SW_START_Y 2.826

void initialize();
void test_init();
void test_loop();
#include "Test.h"

void setup() {
    Serial.begin(9600);
    myPacketSerial.begin(9600);
    myPacketSerial.setPacketHandler(&onPacketReceived);//受信用の関数を割当て
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    delay(100);
    // Serial.println("ok");
    // test1_init();
    // return;  // returnで本番のプログラムに行かないようにしてる
    initialize();
    // リフトは上げとく
    air_lift.write(AIR_LIFT_UP);
    // アームは閉じとく
    air_catch.write(AIR_CATCH_ON);
}

void loop() {
    static State prev_state_ = state;
    static int n = 0;
    static int led = 0;
    static unsigned long end_ts = 0;
    // test1_loop();
    // return;
    // 変数
    bool pushed = false;
    // オドメトリを取得
    Pos odom_pos = odom.get();
    // 座標変換
    Pos robot_pos = conv.get(odom_pos);

    switch (state) {
        case WAIT_START:  // ボタンが押されるのを待っている
            if (sw_tact1.isPushed()) {
                pushed = true;
                table_zone = 1;
            } else if (sw_tact2.isPushed()) {
                pushed = true;
                table_zone = 2;
            } else if (sw_tact3.isPushed()) {
                pushed = true;
                table_zone = 3;
            }
            // どれかのボタンが押されたら
            if (pushed) {
                state = MOVE_START_2_CATCH;
                pure_pursuit.setPlan(0, false);
#ifdef USE_TOF
                // Xを補正
                init_pos = correct.collectPos(init_pos, 3.443, true);
                // Yを補正
                init_pos = correct.collectPos(init_pos, 0.0, false);
#endif
                // 表示
                init_pos.print();
                // 修正
                conv.init(odom_pos, init_pos);
            }
            break;
        case MOVE_START_2_CATCH:  // 受け渡し場所へ行く途中
            // ゴールに到着したとき　または　ロボットがゴール付近にいてスイッチが押されたとき
            digitalWrite(PIN_LED3, HIGH);
            if (pure_pursuit.pursuit(robot_pos) || (robot_pos.y > SW_START_Y && sw_catch.isPushed()) ) {
                digitalWrite(PIN_LED3, LOW);
                state = WAIT_CATCH;
            }
            break;
        case WAIT_CATCH:  // 受け渡し場所で待つ
            if (sw_catch.isPushed()) {
                // 手動機が離れるのを待つ
                delay(AIR_CATCH_DELAY);
                state = MOVE_CATCH_2_TABLE;
                pure_pursuit.setPlan(table_zone, false);  // 追従する経路をセット
                // tofで補正．近距離しか見えないので使えるかわからない
                // Xを補正
                // robot_pos = correct.collectPos(robot_pos, 3.443, true);
                // Yを補正
                // robot_pos = correct.collectPos(robot_pos, 1.538, false);
                // 表示
                // robot_pos.print();
                // 修正
                // conv.init(odom_pos, robot_pos);
            }
            break;
        case MOVE_CATCH_2_TABLE:  // テーブルゾーンへ移動中
            digitalWrite(PIN_LED3, HIGH);
            // 配達完了
            if (pure_pursuit.pursuit(robot_pos)) {
                digitalWrite(PIN_LED3, LOW);
                // リフトを下げる
                air_lift.write(AIR_LIFT_DOWN);
                delay(AIR_LIFT_DOWN_DELAY);
                // 離す
                air_catch.write(AIR_CATCH_OFF);
                delay(AIR_RELEASE_DELAY);
                // 終了
                switch (table_zone) {
                    case 1:
#ifdef USE_TOF
                        // Yを補正
                        robot_pos = correct.collectPos(robot_pos, 1.5, false);
                        // 表示
                        robot_pos.print();
                        // 修正
                        conv.init(odom_pos, robot_pos);
#endif
                        state = MOVE_TABLE_2_CATCH;
                        pure_pursuit.setPlan(table_zone, true);  // 追従する帰りの経路をセット
                        break;
                    case 2:
                        // Xを補正．壁が見切れるかも．
                        // robot_pos = correct.collectPos(robot_pos, 0, true);
                        // Yを補正
                        // init_pos = correct.collectPos(init_pos, 1.5, false);
                        // 表示
                        // robot_pos.print();
                        // 修正
                        // conv.init(oedom_pos, robot_pos);
                        state = MOVE_TABLE_2_CATCH;
                        pure_pursuit.setPlan(table_zone, true);  // 追従する帰りの経路をセット
                        break;
                    case 3:
                        omni.setRobotSpeed(0, 1.0, 0, robot_pos.t);
                        end_ts = millis() + BACK_LAST_DELAY;
                        state = END;
                        break;
                }
            }
            break;
        case MOVE_TABLE_2_CATCH:  // 受け渡しゾーンへ移動中
            //  リフトを上げながら，アームを閉じる
            if (robot_pos.y > LIFT_Y) {
                air_lift.write(AIR_LIFT_UP);
                air_catch.write(AIR_CATCH_ON);
            }
            // ゴールに到着したとき　または　ロボットがゴール付近にいてスイッチが押されたとき
            if (pure_pursuit.pursuit(robot_pos) || (robot_pos.y > SW_START_Y && sw_catch.isPushed()) ) {  // 受け取りゾーンに到着
                state = WAIT_CATCH;
                // 次のテーブルゾーンにする
                table_zone++;
            }
            break;
        case END:
            if (millis() > end_ts)
                omni.setRobotSpeed(0, 0, 0, robot_pos.t);
            break;
    }
    if (prev_state_ != state) {
        Serial.print("\n");
        Serial.print(int(prev_state_));
        Serial.print(" -> ");
        Serial.println(int(state));
        prev_state_ = state;
    }
    if (++n == 100) {
        led = !led;
        n = 0;
        digitalWrite(PIN_LED1, led);
    }
    delay(5);
}

void initialize() {
    m12.init();
    m34.init();
    encoder.init();
    im.init();
    odom.init();
#ifdef USE_TOF
    tof.init();
#endif
    air_lift.init();
    air_catch.init();
    sw_tact1.init();
    sw_tact2.init();
    sw_tact3.init();
    sw_catch.init();
}

// テスト用
void test_init() {
    //    air_lift.init();
    //    air_catch.init();
    // setup
    // encoder.init();
    //  im.init();
    //  tof.init();
    //    m12.init();
    //    m34.init();
}

void test_loop() {
    // float angle1, angle2;
    // encoder.get_angle(angle1, angle2);
    // Serial.println(angle2);//エンコーダの角度
    //    Serial.println(im.getYaw());//ジャイロセンサの角度
    //    Serial.print(", ");
    //    float d1, d2, d3, d4;
    //    tof.getDistance(0, d1);
    //    tof.getDistance(1, d2);
    //    tof.getDistance(2, d3);
    //    tof.getDistance(3, d4);

    //      m12.setSpeed(1,1);
    //      m34.setSpeed(1,1);//ロボクロー二つをsakuraにつなげる　かっこの中を-1にすると逆転するか確認

    //      air_lift.write(AIR_LIFT_UP);
    // air_lift.write(AIR_LIFT_DOWN);
void onPacketReceived(const uint8_t* buffer, size_t size){//PC（GUI）からteensyに通信
  robomas.setData((float*)buffer, size/4);//imuクラスに目標値を与える
  float tempBuffer[2] = {0, im.getYaw()};//imuの現在の測定値を配列にする
  myPacketSerial.send((uint8_t*)tempBuffer, 4 * 2);//teensyからPC（GUI）にロボマスの現在の測定値を送信 4はfloat(4 byte)の意味で、2は配列の数
} 

    Serial.println();

    delay(50);
    static bool led = false;
    static int i = 0;
    if (++i % 10 == 0) {
        led = !led;
        digitalWrite(PIN_LED1, led);
    }
}
