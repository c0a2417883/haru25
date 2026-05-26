#include "haru25_bt/haru25_bt_ros.hpp"

std::shared_ptr<HaruBt> haru_ros;

// 非同期ノード
class SetPlan : public StatefulActionNode {
   public:
    SetPlan(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config) {}

    // ノードの引数を定義
    static PortsList providedPorts() {
        return {InputPort<int>("path_i")};
    }
    // ノードの最初に１回だけ実行される
    NodeStatus onStart() override {
        auto res = getInput<int>("path_i");
        if(!res){
            RCLCPP_ERROR(haru_ros->get_logger(), "SetPlan: Input Empty");
            return NodeStatus::FAILURE;
        }
        int path_i = res.value();
        haru_ros->log("SetPlan : %d", path_i);
        haru_ros->plan_set(path_i);
        return NodeStatus::RUNNING;//経路追従中
    }
    // ノードが成功するまで繰り返し実行される
    NodeStatus onRunning() override {
        if (haru_ros->plan_isReached()) {
            haru_ros->log("SetPlan : SUCCESS");
            return NodeStatus::SUCCESS;//到着
        }
        return NodeStatus::RUNNING;//経路追従中
    }
    void onHalted() override {}
};

class GetPosY : public StatefulActionNode {
    double min_y_;
    public:
     GetPosY(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config) {}
 
     // ノードの引数を定義
     static PortsList providedPorts() {
         return {InputPort<double>("min_y")};
     }
     // ノードの最初に１回だけ実行される
     NodeStatus onStart() override {
         auto res = getInput<double>("min_y");
         if(!res){
             RCLCPP_ERROR(haru_ros->get_logger(), "GetPosY: Input Empty");
             return NodeStatus::FAILURE;
         }
         min_y_ = res.value();
         haru_ros->log("GetPosY : %.2f", min_y_);
         return NodeStatus::RUNNING;//待ってる
     }
     // ノードが成功するまで繰り返し実行される
     NodeStatus onRunning() override {
         if (haru_ros->pos.y >= min_y_) {
             haru_ros->log("GetPosY : SUCCESS");
             return NodeStatus::SUCCESS;//到着
         }
         return NodeStatus::RUNNING;//経路追従中
     }
     void onHalted() override {}
 };

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    BehaviorTreeFactory factory;
    // 常にSUCESSを返すノード
    factory.registerSimpleAction("Shot", [](const TreeNode &node) { haru_ros->air_write(SHOT, ERASER_SHOT); haru_ros->log("Shot"); return NodeStatus::SUCCESS; });
    factory.registerSimpleAction("CatchBall", [](const TreeNode &node) { haru_ros->air_write(BALLCATCH_RELEASE, BALL_CATCH); haru_ros->log("CatchBall"); return NodeStatus::SUCCESS; });
    factory.registerSimpleAction("ReleaseBall", [](const TreeNode &node) { haru_ros->air_write(BALLCATCH_RELEASE, BALL_RELEASE); haru_ros->log("ReleaseBall"); return NodeStatus::SUCCESS; });
    factory.registerSimpleAction("CatchEraser", [](const TreeNode &node) { haru_ros->air_write(ERASER, ERASER_CATCH); haru_ros->log("CatchEraser"); return NodeStatus::SUCCESS; });
    factory.registerSimpleAction("ReleaseEraser", [](const TreeNode &node) { haru_ros->air_write(ERASER, ERASER_RELEASE); haru_ros->log("ReleaseEraser"); return NodeStatus::SUCCESS; });
    factory.registerSimpleAction("BallDown", [](const TreeNode &node) { haru_ros->air_write(BALLUP_DOWN, BALL_DOWN); haru_ros->log("BallDown"); return NodeStatus::SUCCESS; });
    factory.registerSimpleAction("BallUp", [](const TreeNode &node) { haru_ros->air_write(BALLUP_DOWN, BALL_UP); haru_ros->log("BallUp"); return NodeStatus::SUCCESS; });
    factory.registerSimpleAction("InitPos", [](const TreeNode &node) { haru_ros->init_pos(); haru_ros->log("InitPos"); return NodeStatus::SUCCESS; });

    // 非同期ノード
    factory.registerNodeType<SetPlan>("SetPlan");
    factory.registerNodeType<GetPosY>("GetPosY");

    haru_ros = std::make_shared<HaruBt>(&factory);

    rclcpp::spin(haru_ros);
    haru_ros.reset();
    rclcpp::shutdown();
    return 0;
}