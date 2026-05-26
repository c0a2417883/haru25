#include "haru25_navigation/plan.hpp"

#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <boost/tokenizer.hpp>
#include <cfloat>

Plan::Plan() {}

int Plan::readPlan(std::string &filepath){

     std::string line;
    std::ifstream ifs(filepath);
    // csvを走査
    std::vector<Point2D> path;
    int  col = 0, i = 0;
    while (std::getline(ifs, line)) {     
        // 1行を走査
        boost::tokenizer< boost::escaped_list_separator< char > > tokens(line);
        col++;
        switch(col){
            case 1:
                for(auto &t : tokens){
                    Point2D p;
                    p.x = std::stod(t);
                    path.push_back(p);
                }
                break;
            case 2:
                i=0;
                for(auto &t:tokens) path[i++].y = std::stod(t);
                break;
            case 3:
                i=0;
                for(auto &t:tokens) path[i++].yaw = std::stod(t);
                break;
            case 4:
                i=0;
                for(auto &t:tokens) path[i++].vx = std::stod(t);
                break;
            case 5:
                i=0;
                for(auto &t:tokens) path[i++].vy = std::stod(t);
                break;
            case 6:
                i=0;
                for(auto &t:tokens) path[i++].vyaw = std::stod(t);
                col = 0;
                plan_.push_back(path);
                path.clear();
                break;
        }
    }
    plan_size_ = plan_.size();
    for(auto &path:plan_){
        std::cout << path.size() << std::endl;
    }
    return plan_size_;
}
std::vector<Point2D> Plan::setPlan(int plan_i, bool path_reverse, bool flip) {
    std::vector<Point2D> path;
    if (plan_i >= plan_size_)
        return path;
    plan_i_ = plan_i;
    path_reverse_ = path_reverse;
    path_num_ = plan_[plan_i_].size();
    flip_ = flip ? -1 : 1;
    // std::cout << plan_i << ", " << path_num_ << std::endl;
    path_near_i_ = 0;
    path_rage_i_ = 0;
    
    path.resize(path_num_);
    for(int i=0;i<path_num_;++i){
        getPathPoint(i, path[i]);
    }
    return path;
}
// 最近某探索
bool Plan::getNearPoint(Pose2D &robot_pos, Point2D &p1, Point2D &p2) {
    // 最近の点を探索
    double d, d1=DBL_MAX, d2=DBL_MAX;
    int i1=-1, i2=-1;
    Point2D p;
    for(int i=path_near_i_; i < path_num_; ++i){
        getPathPoint(i, p);
        d = getDistanceSquare(robot_pos, p);
        if(d1 > d){
            d2 = d1;
            i2 = i1;
            d1 = d;
            i1 = i;
        }else if(d2 > d){
            d2 = d;
            i2 = i;
        }
    }
    // swap
    if(i1 > i2){
        int tmp = i1;
        i1 = i2;
        i2 = tmp;
    }
    path_near_i_ = i1;
    if(path_near_i_ >= (path_num_ - 2) || i1 == -1 || i2 == -1){
        return false;
    }
    // swap i1, i2
    if(abs(i1-i2) != 1){
        // std::cout << i1<< ", " << i2 << std::endl;
        i2 = i1+1;
    }
    getPathPoint(i1, p1);
    getPathPoint(i2, p2);
    return true;
}

double Plan::getGoal(Pose2D &pos, Point2D &goal) {
    int i = getPathPoint(path_num_ - 1, goal);
    return sqrtf(getDistanceSquare(pos, goal));
}

int Plan::getPathPoint(int path_i, Point2D &p) {
    int i = getPathIndex(path_i);
    p.x = plan_[plan_i_][i].x*flip_;
    p.y = plan_[plan_i_][i].y;
    p.yaw = plan_[plan_i_][i].yaw*flip_;
    p.vx = plan_[plan_i_][i].vx*flip_;
    p.vy = plan_[plan_i_][i].vy;
    p.vyaw = plan_[plan_i_][i].vyaw*flip_;
    return i;
}
int Plan::getPathIndex(int path_i) {
    if (path_i >= path_num_)
        path_i = (path_num_ - 1);
    else if (path_i < 0)
        path_i = 0;

    int i;
    if (path_reverse_)
        i = path_num_ - path_i - 1;
    else
        i = path_i;
    return i;
}
double Plan::getDistanceSquare(Pose2D &p1, Point2D &p2){
    double dx = p1.xy[0] - p2.x;
    double dy = p1.xy[1] - p2.y;
    return dx * dx + dy * dy;
}
