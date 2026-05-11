#include "piper_task/optimal_step_planner.hpp"

namespace piper {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //



// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

OptimalStepPlanner::OptimalStepPlanner(const OptimalStepPlannerConfig& config) : config_(config) {
    OptimalStepErrorCode config_validation = validate_config(config);
    if(config_validation != OptimalStepErrorCode::OK) {
        ROS_WARN("最优路径规划器配置失败: %s", error_code_to_string(config_validation).c_str());
        is_valid_config_ = false;
    }
    else {
        is_valid_config_ = true;
        ROS_WARN("最优路径规划器配置成功");
    }
}

#define EX(code, msg) case OptimalStepErrorCode::code: return msg;
std::string OptimalStepPlanner::error_code_to_string(const OptimalStepErrorCode code) const {
    switch(code) {
        OPTIMAL_STEP_ERROR_CODE_TABLE
        default: return "Unknown error";
    }
}
#undef EX

OptimalStepErrorCode OptimalStepPlanner::set_config(const OptimalStepPlannerConfig& config) {
    OptimalStepErrorCode config_validation = validate_config(config);
    if(config_validation != OptimalStepErrorCode::OK) {
        ROS_WARN("最优路径规划器配置失败: %s", error_code_to_string(config_validation).c_str());
        is_valid_config_ = false;
        return config_validation;
    }
    else {
        config_ = config;
        is_valid_config_ = true;
        ROS_WARN("最优路径规划器配置成功");
        return OptimalStepErrorCode::OK;
    }
}

OptimalStepErrorCode OptimalStepPlanner::plan(const TaskGroup& task_group, OptimalStepPlannerResult& result) {
    if(!is_valid_config_) return OptimalStepErrorCode::INVALID_CONFIG;
    if(is_planning_) return OptimalStepErrorCode::IS_PLANNNING;
    if(validate_task_group(task_group) != OptimalStepErrorCode::OK) return OptimalStepErrorCode::NO_TASK_GROUP;

    is_planning_ = true;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

OptimalStepErrorCode OptimalStepPlanner::validate_config(const OptimalStepPlannerConfig& config) {
    if(config.arm_workspace.radius <= 0) return OptimalStepErrorCode::INVALID_CONFIG;
    if(config.step_sampling.min_step_m <= 0 || config.step_sampling.max_step_m <= 0 || config.step_sampling.resolution_m <= 0) return OptimalStepErrorCode::INVALID_CONFIG;
    if(config.step_sampling.min_step_m > config.step_sampling.max_step_m) return OptimalStepErrorCode::INVALID_CONFIG;
    if(config.rating_weights.valid_count < 0 || config.rating_weights.center_distance < 0 || config.rating_weights.step_cost < 0) return OptimalStepErrorCode::INVALID_CONFIG;
    if(config.forward_axis != OptimalStepPlannerConfig::Axis::X && config.forward_axis != OptimalStepPlannerConfig::Axis::Y && config.forward_axis != OptimalStepPlannerConfig::Axis::Z) return OptimalStepErrorCode::INVALID_CONFIG;

    return OptimalStepErrorCode::OK;
}

OptimalStepErrorCode OptimalStepPlanner::validate_task_group(const TaskGroup& task_group) {
    if(task_group.tasks.empty()) return OptimalStepErrorCode::NO_TASK_GROUP;

    return OptimalStepErrorCode::OK;
}

}
