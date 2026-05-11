#include "piper_task/optimal_step_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <ros/ros.h>

namespace piper {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

double point_distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

geometry_msgs::Point apply_candidate_step(
    const geometry_msgs::Point& point,
    const double step,
    const OptimalStepPlannerConfig::Axis axis
) {
    geometry_msgs::Point predicted_point = point;

    switch(axis) {
        case OptimalStepPlannerConfig::Axis::X:
            predicted_point.x -= step;
            break;
        case OptimalStepPlannerConfig::Axis::Y:
            predicted_point.y -= step;
            break;
        case OptimalStepPlannerConfig::Axis::Z:
            predicted_point.z -= step;
            break;
    }

    return predicted_point;
}

} /* namespace */

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
    if(validate_task_group(task_group) != OptimalStepErrorCode::OK) return OptimalStepErrorCode::NO_TASK_GROUP;
    if(is_planning_.exchange(true)) return OptimalStepErrorCode::IS_PLANNNING;

    result = OptimalStepPlannerResult{};

    const TaskGroup valid_task_group = build_valid_task_group(task_group);
    result.ordered_task_group = build_ordered_task_group(valid_task_group);
    result.invalid_task_group = build_invalid_task_group(task_group);

    const auto add_candidate_evaluation = [this, &result](const double step) {
        OptimalStepPlannerResult::CandidateStepEvaluation evaluation;
        evaluation.step = step;
        evaluation.rating_score = rate_candidate_step(result.invalid_task_group, step);
        result.candidate_evaluations.push_back(evaluation);
        };

    const double min_step = config_.step_sampling.min_step_m;
    const double max_step = config_.step_sampling.max_step_m;
    const double resolution = config_.step_sampling.resolution_m;

    double last_step = min_step;
    for(double step = min_step; step <= max_step + 1e-9; step += resolution) {
        add_candidate_evaluation(step);
        last_step = step;
    }

    if(result.candidate_evaluations.empty() || std::abs(last_step - max_step) > 1e-6) {
        add_candidate_evaluation(max_step);
    }

    if(result.candidate_evaluations.empty()) {
        is_planning_ = false;
        return OptimalStepErrorCode::PLANNING_FAILED;
    }

    std::sort(
        result.candidate_evaluations.begin(),
        result.candidate_evaluations.end(),
        [](const auto& lhs, const auto& rhs) {
            if(std::abs(lhs.rating_score - rhs.rating_score) > 1e-9) {
                return lhs.rating_score > rhs.rating_score;
            }
            return lhs.step < rhs.step;
        });

    result.best_step = result.candidate_evaluations.front().step;
    result.best_rating_score = result.candidate_evaluations.front().rating_score;

    is_planning_ = false;
    return OptimalStepErrorCode::OK;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

OptimalStepErrorCode OptimalStepPlanner::validate_config(const OptimalStepPlannerConfig& config) {
    if(config.arm_workspace.radius <= 0) return OptimalStepErrorCode::INVALID_CONFIG;
    if(config.step_sampling.min_step_m < 0 || config.step_sampling.max_step_m < 0 || config.step_sampling.resolution_m <= 0) return OptimalStepErrorCode::INVALID_CONFIG;
    if(config.step_sampling.min_step_m > config.step_sampling.max_step_m) return OptimalStepErrorCode::INVALID_CONFIG;
    if(config.rating_weights.valid_count < 0 || config.rating_weights.center_distance < 0 || config.rating_weights.step_cost < 0) return OptimalStepErrorCode::INVALID_CONFIG;
    if(config.forward_axis != OptimalStepPlannerConfig::Axis::X && config.forward_axis != OptimalStepPlannerConfig::Axis::Y && config.forward_axis != OptimalStepPlannerConfig::Axis::Z) return OptimalStepErrorCode::INVALID_CONFIG;

    return OptimalStepErrorCode::OK;
}

OptimalStepErrorCode OptimalStepPlanner::validate_task_group(const TaskGroup& task_group) {
    if(task_group.tasks.empty()) return OptimalStepErrorCode::NO_TASK_GROUP;

    return OptimalStepErrorCode::OK;
}

tl::optional<geometry_msgs::Point> OptimalStepPlanner::extract_task_point(const Task& task) const {
    if(!task.target) return tl::nullopt;

    const TargetVariant& target = task.target.value();
    return std::visit(variant_visitor{
        [](const std::monostate&) -> tl::optional<geometry_msgs::Point> {
            return tl::nullopt;
        },
        [](const geometry_msgs::Pose& pose) -> tl::optional<geometry_msgs::Point> {
            return tl::make_optional(pose.position);
        },
        [](const geometry_msgs::Point& point) -> tl::optional<geometry_msgs::Point> {
            return tl::make_optional(point);
        },
        [](const geometry_msgs::Quaternion&) -> tl::optional<geometry_msgs::Point> {
            return tl::nullopt;
        },
        [](const geometry_msgs::PoseStamped& pose_stamped) -> tl::optional<geometry_msgs::Point> {
            return tl::make_optional(pose_stamped.pose.position);
        }
        }, target);
}

bool OptimalStepPlanner::is_in_arm_workspace(const geometry_msgs::Point& point) const {
    return point_distance(point, config_.arm_workspace.center) <= config_.arm_workspace.radius;
}

TaskGroup OptimalStepPlanner::build_valid_task_group(const TaskGroup& task_group) {
    TaskGroup valid_group;
    valid_group.go_home_after_finish = task_group.go_home_after_finish;
    valid_group.sort_type = task_group.sort_type;
    valid_group.weight_orient = task_group.weight_orient;

    for(const auto& [id, task] : task_group.tasks) {
        const auto point = extract_task_point(task);
        if(!point) continue;

        if(is_in_arm_workspace(*point)) {
            valid_group.tasks.emplace(id, task);
        }
    }

    return valid_group;
}

TaskGroup OptimalStepPlanner::build_invalid_task_group(const TaskGroup& task_group) {
    TaskGroup invalid_group;
    invalid_group.go_home_after_finish = task_group.go_home_after_finish;
    invalid_group.sort_type = task_group.sort_type;
    invalid_group.weight_orient = task_group.weight_orient;

    for(const auto& [id, task] : task_group.tasks) {
        const auto point = extract_task_point(task);
        if(!point) {
            invalid_group.tasks.emplace(id, task);
            continue;
        }

        if(!is_in_arm_workspace(*point)) {
            invalid_group.tasks.emplace(id, task);
        }
    }

    return invalid_group;
}

TaskGroup OptimalStepPlanner::build_ordered_task_group(const TaskGroup& task_group) {
    TaskGroup ordered_group = task_group;
    ordered_group.sorted_tasks.clear();

    std::vector<unsigned int> remaining_ids;
    remaining_ids.reserve(task_group.tasks.size());

    for(const auto& [id, task] : task_group.tasks) {
        if(extract_task_point(task)) remaining_ids.push_back(id);
    }

    geometry_msgs::Point current = config_.arm_workspace.center;

    while(!remaining_ids.empty()) {
        auto best_it = remaining_ids.begin();
        double best_dist = std::numeric_limits<double>::max();

        for(auto it = remaining_ids.begin(); it != remaining_ids.end(); ++it) {
            const Task& candidate = task_group.tasks.at(*it);
            const auto point = extract_task_point(candidate);
            if(!point) continue;

            const double dist = point_distance(current, *point);
            if(dist < best_dist) {
                best_dist = dist;
                best_it = it;
            }
        }

        const unsigned int selected_id = *best_it;
        Task selected_task = task_group.tasks.at(selected_id);
        selected_task.id = selected_id;
        ordered_group.sorted_tasks.push_back(selected_task);

        const auto selected_point = extract_task_point(selected_task);
        if(selected_point) current = *selected_point;

        remaining_ids.erase(best_it);
    }

    return ordered_group;
}

double OptimalStepPlanner::rate_candidate_step(const TaskGroup& task_group, double step) {
    std::size_t valid_count = 0;
    double center_distance_sum = 0.0;

    for(const auto& [id, task] : task_group.tasks) {
        (void)id;

        const auto point = extract_task_point(task);
        if(!point) continue;

        const geometry_msgs::Point predicted_point = apply_candidate_step(*point, step, config_.forward_axis);
        const double center_distance = point_distance(predicted_point, config_.arm_workspace.center);
        if(center_distance <= config_.arm_workspace.radius) {
            ++valid_count;
            center_distance_sum += center_distance;
        }
    }

    if(valid_count == 0) return -config_.rating_weights.step_cost * step;

    const double avg_center_distance = center_distance_sum / static_cast<double>(valid_count);

    return config_.rating_weights.valid_count * static_cast<double>(valid_count)
        - config_.rating_weights.center_distance * avg_center_distance
        - config_.rating_weights.step_cost * step;
}

} /* namespace piper */
