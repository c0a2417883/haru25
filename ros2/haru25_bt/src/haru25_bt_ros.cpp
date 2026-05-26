#include "haru25_bt/haru25_bt_ros.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

HaruBt::HaruBt(BehaviorTreeFactory *factory) : Node("haru25_bt"), bt_factory_(factory) {
    // パラメータの読み込み
    this->declare_parameter("bt_path", "/home/yui/ros2_ws/src/haru25/ros2/haru25/config/haru25.xml");
    this->get_parameter("bt_path", bt_path_);
    double x, y, yaw;
    this->declare_parameter("init_pos_x", 2.983);
    this->get_parameter("init_pos_x", x);
    this->declare_parameter("init_pos_y", 0.25);
    this->get_parameter("init_pos_y", y);
    this->declare_parameter("init_pos_yaw", 0.0);
    this->get_parameter("init_pos_yaw", yaw);
    this->declare_parameter("max_a", 3.0);
    this->get_parameter("max_a", max_a_);
    init_pos_.x = x;
    init_pos_.y = y;
    init_pos_.yaw = yaw;

    // Air
    haru25_msgs::msg::CAN can4, can5;
    can4.id = 4;
    can4.data.push_back(0);
    can_array_.array.push_back(can4);

    // led_tape
    can5.id = 5;
    can5.data.push_back(0);
    can_array_.array.push_back(can5);

    plan_pub_ = this->create_publisher<haru25_msgs::msg::Plan>("plan", 10);
    can_pub_ = this->create_publisher<haru25_msgs::msg::CANArray>("can/tx", 10);
    init_pub_ = this->create_publisher<haru25_msgs::msg::Pos>("init_pos", 10);

    plan_sub_ = this->create_subscription<haru25_msgs::msg::Plan>("reach", 10, std::bind(&HaruBt ::plan_cb, this, _1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&HaruBt ::joy_cb, this, _1));
    pos_sub_ = this->create_subscription<haru25_msgs::msg::Pos>("pos", 10, std::bind(&HaruBt ::pos_cb, this, _1));
    
    timer_ = this->create_wall_timer(10ms, std::bind(&HaruBt::timer_cb, this));

    // 仮にツリーを作成しておく
    bt_factory_->registerBehaviorTreeFromFile(bt_path_);
    make_tree("main");

    // webguiが立ち上がるまで待つ
    rclcpp::WallRate loop_rate(2000ms);
    loop_rate.sleep();

    log("change wait");
    log("select cote color");
    air_write(BALLCATCH_RELEASE, BALL_CATCH);
    air_write(BALLUP_DOWN, BALL_UP);
    air_write(SHOT, false);
    air_write(ERASER, ERASER_RELEASE);
}

void HaruBt::make_tree(std::string tree_name) {
    // ビヘイビアツリーの設定ファイルを読み込み
    bt_tree_ = std::make_unique<Tree>(bt_factory_->createTree(tree_name));
    // Groot2へ出力
    // bt_groot_pub_.reset();
    // bt_groot_pub_ = std::make_unique<Groot2Publisher>(*bt_tree_, 5555);
}

void HaruBt::timer_cb() {
    if(joy_ == nullptr) return;
    static auto m_start = std::chrono::system_clock::now();
    auto m_now = std::chrono::system_clock::now();
    unsigned long  nowtimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(m_now - m_start).count();
    static unsigned long push_manual_time_stamp = 0;
    static unsigned long push_red_time_stamp = 0;
    static unsigned long push_start_time_stamp = 0;

    //手動, 待機切り替え
    if((nowtimestamp - push_manual_time_stamp) > 300){
        if(joy_->buttons[MENU]){    
            if(state_ == WAIT || state_ == AUTO){
                push_manual_time_stamp = nowtimestamp;
                state_ = MANUAL;
                manual_yaw_ = pos.yaw;//いきなり回転するのを防ぐため
                int data = can_array_.array[0].data[0];
                air_a_ = bitRead(data, BALLCATCH_RELEASE);
                air_b_ = bitRead(data, ERASER);
                air_x_ = bitRead(data, SHOT);
                air_y_ = bitRead(data, BALLUP_DOWN);
                log("chenge manual");
            }else{
                push_manual_time_stamp = nowtimestamp;
                state_ = WAIT;
                log("chenge wait");
            }
        }
    }

    // 待機モードのとき
    //自動運転スタート
    if(state_ == WAIT || state_ == AUTO){
        if((nowtimestamp-push_start_time_stamp) > 300){
            // リスタート処理
            if(joy_->axes[UP_DWUN] > 0){
                // ツリーを生成してリスタート
                state_ = AUTO; 
                log("change auto & make tree");
                push_start_time_stamp = nowtimestamp;
                // BehaviorTreeの初期化
                make_tree("main");
            }else if(joy_->axes[UP_DWUN] < 0){
                // リスタート用ツリーを生成
                state_ = AUTO; 
                log("change restart auto & make restart tree");
                push_start_time_stamp = nowtimestamp;
                make_tree("restart");
            }else if(joy_->axes[LEFT_RIGHT] > 0){
                // リスタート用ツリーを生成
                state_ = AUTO; 
                log("change auto");
                push_start_time_stamp = nowtimestamp;
            }
        }
    }

    
    if(state_ == WAIT){
        //コート切り替え
        if((nowtimestamp-push_red_time_stamp) > 300){
            if(joy_->buttons[SCREEN]){
                red_ = !red_;    
                init_pos(); 
                push_red_time_stamp = nowtimestamp;
                if(red_){
                    log("chenge red");
                }else{
                    log("chenge blue");
                }
            }
        }
    }

    // BehaviorTreeの実行
    static NodeStatus status;
    haru25_msgs::msg::Plan plan;
    switch(state_){
        case WAIT:
            // 停止させとく
            plan.mode = haru25_msgs::msg::Plan::STOP;
            plan_pub_->publish(plan);
            break;
        case AUTO:
            if (BT::isStatusCompleted(status)) {
                RCLCPP_WARN(this->get_logger(), "BT is completed !");
                state_ = MANUAL;
                manual_yaw_ = pos.yaw;//いきなり回転するのを防ぐため
                log("manual start");
            } else {
                status = bt_tree_->tickOnce();//BehaviorTreeのノードを1回実行
            }
            break;
        case MANUAL:
            manual();
            break;
        default:
            break;
    }
    // 機構のCAN通信
    can_pub_->publish(can_array_);
}

void HaruBt::manual(){
    if(joy_ == nullptr) return;
    
    auto plan = haru25_msgs::msg::Plan();
    static auto m_start = std::chrono::system_clock::now();
    auto m_now = std::chrono::system_clock::now();
    unsigned long  nowtimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(m_now - m_start).count();

    //x軸移動
    // plan.pos.vx = -(joy_->axes[LX]);
    // if(joy_->axes[LEFT_RIGHT] < 0){
    //     plan.pos.vx = VX_MAX;
    // }else if(joy_->axes[LEFT_RIGHT] > 0){
    //     plan.pos.vx = -VX_MAX;
    // }
    static float prev_vx = 0;
    static float vx = 0;
    static float dvx = 0;
    float a = max_a_;
    float dt = 0.01;//timer_cbの周期
    vx = -(joy_->axes[LX]);
    if(joy_->axes[LEFT_RIGHT] < 0){
        vx = VX_MAX;
    }else if(joy_->axes[LEFT_RIGHT] > 0){
        vx = -VX_MAX;
    }
    dvx = vx - prev_vx;
    dvx = constrain(dvx, -a*dt, a*dt);
    vx = prev_vx + dvx;
    plan.pos.vx = vx;
    prev_vx = vx;

    //y軸移動
    // plan.pos.vy = joy_->axes[LY];
    // if(joy_->axes[UP_DWUN] > 0){
    //     plan.pos.vy = VY_MAX;
    // }else if(joy_->axes[UP_DWUN] < 0){
    //     plan.pos.vy = -VY_MAX;
    // }
    static float prev_vy = 0;
    static float vy = 0;
    static float dvy = 0;
    vy = joy_->axes[LY];
    if(joy_->axes[UP_DWUN] > 0){
        vy = VY_MAX;
    }else if(joy_->axes[UP_DWUN] < 0){
        vy = -VY_MAX;
    }
    dvy = vy - prev_vy;
    dvy = constrain(dvy, -a*dt, a*dt);
    vy = prev_vy + dvy;
    plan.pos.vy = vy;
    prev_vy = vy;

    //LB,RBで旋回動作
    if(joy_->buttons[LB]){
        manual_yaw_ += OMEGAMAX * DT;
    }else if(joy_->buttons[RB]){
        manual_yaw_ -= OMEGAMAX * DT;
    }

    //LT,RTで90度回転
    static unsigned long push_90_time_stamp = 0;
    // unsigned long nowtimestamp = 0.01;//現在時刻をミリ秒で取得
    if( (nowtimestamp-push_90_time_stamp) > 1000 ){//ボタンをおした瞬間だけみる チャタリング防止

        if(joy_->axes[LT] < 0){
            manual_yaw_ += M_PI/2;
            push_90_time_stamp = nowtimestamp;
        }else if(joy_->axes[RT] < 0){
            manual_yaw_ += -M_PI/2;
            push_90_time_stamp = nowtimestamp;
        }
    }

    //Air4つ トグルスイッチ
    static unsigned long push_a_time_stamp = 0;
    static unsigned long push_b_time_stamp = 0;
    static unsigned long push_x_time_stamp = 0;
    static unsigned long push_y_time_stamp = 0;

    //tape_led トグルスイッチ
    static unsigned long push_led_time_stamp = 0;

    if((nowtimestamp-push_a_time_stamp) > 1000){
        if(joy_->buttons[A]){
            air_a_ = !air_a_;  
            air_write(BALLUP_DOWN,air_a_);//ros2 topic echo /can/tx idは2進数なので1と表示される
            push_a_time_stamp = nowtimestamp;
        }
        
    }

    if((nowtimestamp-push_b_time_stamp) > 1000){
        if(joy_->buttons[B]){
            air_b_ = !air_b_;  
            air_write(BALLCATCH_RELEASE,air_b_);
            push_b_time_stamp = nowtimestamp;
        }
        
    }
    if((nowtimestamp-push_x_time_stamp) > 1000){
        if(joy_->buttons[X]){
            air_x_ = !air_x_;
            air_write(SHOT,air_x_);
            push_x_time_stamp = nowtimestamp;
        }
    }

    if((nowtimestamp-push_y_time_stamp) > 1000){
        if(joy_->buttons[Y]){
            air_y_ = !air_y_;
            air_write(ERASER,air_y_);
            push_y_time_stamp = nowtimestamp;
        }
    }

    if((nowtimestamp-push_led_time_stamp) > 1000){
        if(joy_->buttons[R3]){
            led_ = !led_;
            led_write(GREEN);
            push_led_time_stamp = nowtimestamp;
        }
    }


    plan.pos.yaw = manual_yaw_;
    plan.mode = haru25_msgs::msg::Plan::MANUAL;
    plan_pub_->publish(plan);
}

void HaruBt::joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg){
    joy_ = msg;
}

void HaruBt::air_write(uint8_t id, bool on){
    int data = can_array_.array[0].data[0];
    bitWrite(data, id, on);
    can_array_.array[0].data[0] = data;
}

void HaruBt::led_write(int color){
    can_array_.array[1].data[0] = color;
}

void HaruBt::log(const char *str, ...) {
    va_list arg;
    va_start(arg, str);
    char c[70];
    vsprintf(c, str, arg);
    RCLCPP_INFO(this->get_logger(), c);
    va_end(arg);
}

// 自動運転
void HaruBt::plan_set(int path_i){
    plan_isReached_ = false;
    auto plan = haru25_msgs::msg::Plan();
    plan.mode = haru25_msgs::msg::Plan::AUTO;
    plan.flip = !red_;
    // 反対かつ経路３のとき
    if (plan.flip && path_i==3){
        path_i = 4; // 専用パス
    }
    plan.path_i = path_i;
    // 自動運転オフ
    plan_pub_->publish(plan);
}
void HaruBt::plan_cb(const haru25_msgs::msg::Plan::SharedPtr msg ){
    plan_isReached_ = true;
}
bool HaruBt::plan_isReached(){
    return plan_isReached_;
}

void HaruBt::init_pos(){
    haru25_msgs::msg::Pos pos;
    pos.y = init_pos_.y;
    if(red_){
        pos.x = init_pos_.x;
        pos.yaw = init_pos_.yaw;
    }else{
        pos.x = -init_pos_.x;
        pos.yaw = -init_pos_.yaw;
    }
    init_pub_->publish(pos);
}

void HaruBt::pos_cb(const haru25_msgs::msg::Pos::SharedPtr msg){
    pos.x = msg->x;
    pos.y = msg->y;
    pos.yaw = msg->yaw;
    pos.vx = msg->vx;
    pos.vy = msg->vy;
    pos.vyaw = msg->vyaw;
}