#pragma once

#include <geometry_msgs/Point.h>

#include "piper_task/tasks_manager.hpp"

namespace piper {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

#define OPTIMAL_STEP_ERROR_CODE_TABLE \
    EX(OK, "OK") \
    EX(INVALID_PARAMS, "Invalid parameters") \
    EX(INVALID_CONFIG, "Invalid configuration") \
    EX(NO_TASK_GROUP, "No task group provided") \
    EX(IS_PLANNNING, "Planning is already in progress") \
    EX(PLANNING_FAILED, "Planning failed")

#define EX(code, msg) code,
enum class OptimalStepErrorCode : uint8_t {
    OPTIMAL_STEP_ERROR_CODE_TABLE
};
#undef EX

struct OptimalStepPlannerConfig {
    struct ArmWorkspace {
        geometry_msgs::Point center{};
        double radius{};
    } arm_workspace{};

    struct StepSampling {
        double min_step_m{};
        double max_step_m{};
        double resolution_m{};
    } step_sampling{};

    struct RatingWeights {
        double valid_count{};
        double center_distance{};
        double step_cost{};
    } rating_weights{};

    enum class Axis {
        X,
        Y,
        Z
    } forward_axis{};
};

struct OptimalStepPlannerResult {
    TaskGroup ordered_task_group{};
    TaskGroup invalid_task_group{};

    struct CandidateStepEvaluation {
        double step{};
        double rating_score{};
    };
    std::vector<CandidateStepEvaluation> candidate_evaluations{};

    double best_rating_score{};
    double best_step{};
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class OptimalStepPlanner {
public:
    explicit OptimalStepPlanner(const OptimalStepPlannerConfig& config);
    ~OptimalStepPlanner() = default;

    OptimalStepPlanner(const OptimalStepPlanner&) = delete;
    OptimalStepPlanner& operator=(const OptimalStepPlanner&) = delete;
    OptimalStepPlanner(OptimalStepPlanner&&) = delete;
    OptimalStepPlanner& operator=(OptimalStepPlanner&&) = delete;

    std::string error_code_to_string(const OptimalStepErrorCode code) const;

    OptimalStepErrorCode set_config(const OptimalStepPlannerConfig& config);
    OptimalStepErrorCode plan(const TaskGroup& task_group, OptimalStepPlannerResult& result);

private:
    OptimalStepErrorCode validate_config(const OptimalStepPlannerConfig& config);
    OptimalStepErrorCode validate_task_group(const TaskGroup& task_group);

private:
    OptimalStepPlannerConfig config_;

    bool is_valid_config_{ false };
    std::atomic<bool> is_planning_{ false };
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

}
