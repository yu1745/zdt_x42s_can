# ESP-IDF project: esp32c3_motor_test
# 真机测试：esp32c3 + CAN 收发器 + ZDT_X42S 电机
#
# 本工程位于仓库 examples/ 下，仓库根即组件根（zdt_can 组件直接在仓库根），
# 用 EXTRA_COMPONENT_DIRS 指向仓库根即可被解析到。
cmake_minimum_required(VERSION 3.16)

# 仓库根 = 本 CMakeLists.txt 往上两级：esp32c3_motor_test -> examples -> zdt_x42s_can
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(EXTRA_COMPONENT_DIRS "${REPO_ROOT}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32c3_motor_test)
