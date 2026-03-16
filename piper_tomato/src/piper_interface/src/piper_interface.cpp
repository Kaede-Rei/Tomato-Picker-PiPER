#include "piper_interface/piper_interface.hpp"

namespace piper {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

ROSInterface::ROSInterface(ros::NodeHandle& nh, const ROSInterfaceConfig& config) {
    _arm_ = std::make_shared<ArmController>(config.arm_group_name);
    init_interfaces(nh, config);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

void ROSInterface::init_interfaces(ros::NodeHandle& nh, const ROSInterfaceConfig& config) {
    add_interface<ArmMoveAction>(config.arm_move_action, nh, _arm_, config.arm_move_action.name);
    add_interface<SimpleArmMoveAction>(config.simple_arm_move_action, nh, _arm_, config.simple_arm_move_action.name);
    add_interface<ArmConfigService>(config.arm_config_service, nh, _arm_, config.arm_config_service.name);
    add_interface<ArmQueryService>(config.arm_query_service, nh, _arm_, config.arm_query_service.name);
}

}
