#pragma once

#include <geometry_msgs/Point.h>

#include "piper_task/tasks_manager.hpp"

namespace piper {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class OptimalStepErrorCode {
    OK,
    INVAILD_PARAMS,
    PLANNING_FAILED,
};

struct OptimalStepPlannerConfig {
    struct ArmWorkspace {
        geometry_msgs::Point center{};
        double radius{};
    };
};

struct OptimalStepPlannerResult {
    TaskGroup ordered_task_group{};
    TaskGroup invaild_task_group{};
    std::vector<double> candidate_steps{};
    std::vector<double> candidate_rating_score{};
    double best_rating_score{};
    double best_step{};
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class OptimalStepPlanner {
public:
    OptimalStepPlanner() = default;
    ~OptimalStepPlanner() = default;

    OptimalStepErrorCode set_config(const OptimalStepPlannerConfig& config);
    OptimalStepErrorCode add_task_group(const TaskGroup& task_group);
    OptimalStepErrorCode plan(OptimalStepPlannerResult& result);
private:
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

}
