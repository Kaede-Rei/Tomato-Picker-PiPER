#include "piper_interface/arm_interface.hpp"

namespace piper {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

ArmAction::ArmAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name)
    : _arm_(std::move(arm)), _dispatcher_(std::move(dispatcher)) {
    _as_ = std::make_unique<MoveArmAS>(nh, action_name, false);
    _as_->registerGoalCallback(boost::bind(&ArmAction::on_goal, this));
    _as_->registerPreemptCallback(boost::bind(&ArmAction::on_preempt, this));
    _as_->start();
}

SimpleArmAction::SimpleArmAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name)
    : _arm_(std::move(arm)), _dispatcher_(std::move(dispatcher)) {
    _as_ = std::make_unique<MoveArmAS>(nh, action_name, false);
    _as_->registerGoalCallback(boost::bind(&SimpleArmAction::on_goal, this));
    _as_->registerPreemptCallback(boost::bind(&SimpleArmAction::on_preempt, this));
    _as_->start();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

bool ArmAction::convert_goal_to_request(const piper_msgs::MoveArmGoal& goal, ArmCmdRequest& req) {
    req.type = static_cast<ArmCmdType>(goal.command_type);
    if(!(ArmCmdType::MIN < req.type && req.type < ArmCmdType::MAX)) {
        ROS_WARN("接收到无效的命令类型: %d", goal.command_type);
        return false;
    }
    req.joint_names = goal.joint_names;
    req.joints = goal.joints;

    if(goal.target_type == goal.TARGET_POSE) req.target = goal.pose;
    else if(goal.target_type == goal.TARGET_POINT) req.target = goal.point;
    else if(goal.target_type == goal.TARGET_QUATERNION) req.target = goal.quaternion;
    else {
        ROS_WARN("接收到无效的目标类型: %d", goal.target_type);
        return false;
    }

    req.values = goal.values;
    req.waypoints = goal.waypoints;

    return true;
}

void ArmAction::on_goal() {
    auto goal = _as_->acceptNewGoal();
    if(!goal) {
        piper_msgs::MoveArmResult res;
        res.success = false;
        res.message = "空目标";
        _as_->setAborted(res, res.message);

        return;
    }

    ArmCmdRequest req;
    if(!convert_goal_to_request(*goal, req)) {
        piper_msgs::MoveArmResult res;
        res.success = false;
        res.message = "无效的目标";
        _as_->setAborted(res, res.message);

        return;
    }

    auto result = _dispatcher_->dispatch(req, [this](const ArmCmdFeedback& fb) {
        piper_msgs::MoveArmFeedback feedback;
        feedback.stage = fb.stage;
        feedback.progress = fb.progress;
        feedback.message = fb.message;
        this->_as_->publishFeedback(feedback);
        });

    piper_msgs::MoveArmResult res;
    res.success = result.success;
    res.message = result.message;
    res.error_code = static_cast<uint8_t>(result.error_code);
    res.cur_joint = result.current_joints;
    res.cur_pose = result.current_pose;
    res.values = result.values;

    if(result.success) _as_->setSucceeded(res);
    else _as_->setAborted(res, res.message);
}

void ArmAction::on_preempt() {
    _dispatcher_->cancel();
    piper_msgs::MoveArmResult res;
    res.success = false;
    res.message = "目标被取消";
    _as_->setPreempted(res, res.message);
}

bool SimpleArmAction::convert_goal_to_request(const piper_msgs::SimpleMoveArmGoal& goal, ArmCmdRequest& req) {
    req.type = static_cast<ArmCmdType>(goal.command_type);
    if(!(ArmCmdType::MIN < req.type && req.type < ArmCmdType::MAX)) {
        ROS_WARN("接收到无效的命令类型: %d", goal.command_type);
        return false;
    }
    req.joint_names = goal.joint_names;
    req.joints = goal.joints;

    if(goal.target_type == goal.TARGET_POSE) {
        geometry_msgs::Pose pose;
        pose.position.x = goal.x[0];
        pose.position.y = goal.y[0];
        pose.position.z = goal.z[0];
        tf2::Quaternion quat;
        quat.setRPY(goal.roll[0], goal.pitch[0], goal.yaw[0]);
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        req.target = pose;
    }
    else if(goal.target_type == goal.TARGET_POINT) {
        geometry_msgs::Point point;
        point.x = goal.x[0];
        point.y = goal.y[0];
        point.z = goal.z[0];
        req.target = point;
    }
    else if(goal.target_type == goal.TARGET_ORIENTATION) {
        tf2::Quaternion quat;
        quat.setRPY(goal.roll[0], goal.pitch[0], goal.yaw[0]);
        geometry_msgs::Quaternion orientation;
        orientation.x = quat.x();
        orientation.y = quat.y();
        orientation.z = quat.z();
        orientation.w = quat.w();
        req.target = orientation;
    }
    else {
        ROS_WARN("接收到无效的目标类型: %d", goal.target_type);
        return false;
    }

    req.values = goal.values;
    for(size_t i = 1; i < goal.x.size(); ++i) {
        geometry_msgs::Pose waypoint;
        waypoint.position.x = goal.x[i];
        waypoint.position.y = goal.y[i];
        waypoint.position.z = goal.z[i];
        tf2::Quaternion quat;
        quat.setRPY(goal.roll[i], goal.pitch[i], goal.yaw[i]);
        waypoint.orientation.x = quat.x();
        waypoint.orientation.y = quat.y();
        waypoint.orientation.z = quat.z();
        waypoint.orientation.w = quat.w();
        req.waypoints.push_back(waypoint);
    }

    return true;
}

void SimpleArmAction::on_goal() {
    auto goal = _as_->acceptNewGoal();
    if(!goal) {
        piper_msgs::SimpleMoveArmResult res;
        res.success = false;
        res.message = "空目标";
        _as_->setAborted(res, res.message);

        return;
    }

    ArmCmdRequest req;
    if(!convert_goal_to_request(*goal, req)) {
        piper_msgs::SimpleMoveArmResult res;
        res.success = false;
        res.message = "无效的目标";
        _as_->setAborted(res, res.message);

        return;
    }

    auto result = _dispatcher_->dispatch(req);

    piper_msgs::SimpleMoveArmResult res;
    res.success = result.success;
    res.message = result.message;
    res.error_code = static_cast<uint8_t>(result.error_code);
    res.cur_joint = result.current_joints;

    res.cur_x = result.current_pose.position.x;
    res.cur_y = result.current_pose.position.y;
    res.cur_z = result.current_pose.position.z;
    tf2::Quaternion quat(result.current_pose.orientation.x, result.current_pose.orientation.y, result.current_pose.orientation.z, result.current_pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    res.cur_roll = roll;
    res.cur_pitch = pitch;
    res.cur_yaw = yaw;

    res.values = result.values;

    if(result.success) _as_->setSucceeded(res);
    else _as_->setAborted(res, res.message);
}

void SimpleArmAction::on_preempt() {
    _dispatcher_->cancel();
    piper_msgs::SimpleMoveArmResult res;
    res.success = false;
    res.message = "目标被取消";
    _as_->setPreempted(res, res.message);
}

}
