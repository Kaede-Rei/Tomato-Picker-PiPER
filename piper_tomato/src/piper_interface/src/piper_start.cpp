#include "piper_interface/piper_interface.hpp"

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //



// ! ========================= 私 有 函 数 实 现 ========================= ! //

int main(int argc, char** argv) {
    ros::init(argc, argv, "piper_start");
    ros::NodeHandle nh;

    piper::ROSInterfaceConfig config;
    piper::ROSInterface piper_interface(nh, config);

    return 0;
}
