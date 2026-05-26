#include "haru25_navigation/purepursuit.hpp"
#include <iostream>

PurePursuit::PurePursuit(Plan *plan, PID *pid_yaw, PID *pid_foot, PID *pid_path) : 
    plan_(plan),
    pid_yaw_(pid_yaw),
    pid_foot_(pid_foot),
    pid_path_(pid_path){
}

std::vector<Point2D> PurePursuit::setPlan(int plan_i, bool path_reverse, bool flip) {
     path0_up_ = plan_i==0 ? 1 : 0;
    return plan_->setPlan(plan_i, path_reverse, flip);
}

bool PurePursuit::pursuit(Pose2D &pos, float &vx, float &vy, float &vt) {
    // 最終地点までの距離, 最終地点の角度
    Point2D goal_pos;
    double last_distance, err_angle, v,dx, dy, dl, dl2, rx, ry, p1_rate, p2_rate, ref_yaw;
    last_distance = plan_->getGoal(pos, goal_pos);
    vx = 0;
	vy = 0;
	vt = 0;  // 停止
    if(path0_up_==2){
	vy = path_0_vy_;
	return false;
    }
    
    // 角度の偏差
    err_angle = normalizeAngle(goal_pos.yaw - pos.yaw);
    if (last_distance < path_goal_distance_ && abs(err_angle) < path_goal_angle_) {
    	if(path0_up_ == 0){
    	   // Serial.println("Goal");
	   return true;
    	}else{
	vy = path_0_vy_;
	path0_up_ = 2;
	return false;
    	}
        
    }

    // ロボットと近い経路点を探索
    Point2D near_pos1, near_pos2;
    
    // FF制御
    path_ff_vx = 0;
    path_ff_vy = 0;
    path_ff_vyaw = 0;
    
    // FB制御
    path_fb_v = 0;
    path_fb_vx = 0;
    path_fb_vy = 0;
    path_fb_vyaw = 0;

    if (plan_->getNearPoint(pos, near_pos1, near_pos2) && last_distance > path_pid_distance_) {  // ゴールから離れてる
        if(near_pos1.vx == 0 && near_pos1.vy == 0){
            p2_rate = 1;
        }else if(near_pos2.vx == 0 && near_pos2.vy == 0){
            p2_rate = 0;
        }else{
            // 経路点間のベクトル
            dx = (near_pos2.x - near_pos1.x);
            dy = (near_pos2.y - near_pos1.y);
            dl2 = dx*dx+dy*dy;
            if(dl2 < 0.01 * 0.01){
                dl = 0.01;
                dl2 = 0.01 * 0.01;
            }else{
                dl = sqrt(dl2);
            }
            // 経路点からロボットまでのベクトル
            rx = (pos.xy[0] - near_pos1.x);
            ry = (pos.xy[1] - near_pos1.y);
            //http://www.deqnotes.net/acmicpc/2d_geometry/products
            // |A||B|cosθ/|A|^2 = |B|cosθ/|A|
            p2_rate = (rx * dx + ry * dy) / (dl2);
            // 経路からロボットへの距離（外積）
            // |A||B|sinθ/|A| = |B|sinθ
            path_fb_error = (dx*ry - dy*rx) / (dl);
            path_fb_v = pid_path_->update(-path_fb_error);
            path_fb_vx = -path_fb_v * dy / dl;
            path_fb_vy = path_fb_v * dx / dl;
            // std::cout << path_pid << ", " << path_pid_vx << ", " << path_pid_vy << std::endl;
            // std::cout << path_distance << std::endl;
        }
        p2_rate = constrain(p2_rate, 0, 1);
        p1_rate = 1 - p2_rate;
        // 線形補完
        path_ff_vx = near_pos1.vx*p1_rate + near_pos2.vx*p2_rate;
        path_ff_vy = near_pos1.vy*p1_rate + near_pos2.vy*p2_rate;
        path_ff_vyaw = near_pos1.vyaw*p1_rate + near_pos2.vyaw*p2_rate;
        
        ref_yaw = near_pos1.yaw*p1_rate+near_pos2.yaw*p2_rate;
        // std::cout << last_distance << ", " << e_vx << ", " << e_vy << std::endl;
    } else{
        // 自分の位置からゴールまでの単位ベクトル
        path_fb_vx = (goal_pos.x - pos.xy[0]);
        path_fb_vy = (goal_pos.y - pos.xy[1]);
        v = hypotf(path_fb_vx, path_fb_vy);
        path_fb_vx /= v;
        path_fb_vy /= v;

        path_fb_v  = pid_foot_->update(last_distance);  // 速さを遅くする（ヒョイッと動く）
        
        path_fb_vx *= path_fb_v;
        path_fb_vy *= path_fb_v;
        
        ref_yaw = goal_pos.yaw;
    }
    // FF制御＋FB制御
    vx = path_ff_vx + path_fb_vx;
    vy = path_ff_vy + path_fb_vy;

    // 目標速度の大きさ
    v = hypot(vx, vy);
    if(v < min_vel_linear_ || max_vel_linear_ < v){
        // 目標ベクトルがない
        if(v == 0){
            // ゴールへ
            vx = (goal_pos.x - pos.xy[0]);
            vy = (goal_pos.y - pos.xy[1]);
            v = hypot(vx, vy);
            if(v == 0){
                vx = min_vel_linear_;
                vy = min_vel_linear_;
            }else{
                vx /= v;
                vy /= v;
                v = constrain(v, min_vel_linear_, max_vel_linear_);
                vx *= v;
                vy *= v;
            }
        }else{
            vx /= v;
            vy /= v;
            v = constrain(v, min_vel_linear_, max_vel_linear_);
            vx *= v;
            vy *= v;
        }
    }

    // ロボット全体の角速度(vt)を求める
    err_angle = normalizeAngle(ref_yaw - pos.yaw);
    path_fb_vyaw = pid_yaw_->update(err_angle);// ロボットの角度をaim_tになるようにPD制御
    vt = path_ff_vyaw + path_fb_vyaw;
    vt = constrain(vt, -max_vel_angular_, max_vel_angular_);
    // vt = limit_low(vt, -min_vel_angular_, min_vel_angular_);
    return false;
}

void PurePursuit::setParam(float path_goal_distance,
                            float path_goal_angle,
                            float path_pid_distance,
                            float max_vel_linear,
                            float max_vel_angular,
                            float min_vel_linear,
                            float min_vel_angular) {
    path_goal_distance_ = path_goal_distance;
    path_goal_angle_ = path_goal_angle;
    path_pid_distance_ = path_pid_distance;
    max_vel_linear_ = max_vel_linear;
    max_vel_angular_ = max_vel_angular;
    min_vel_linear_ = min_vel_linear;
    min_vel_angular_ = min_vel_angular;
}
