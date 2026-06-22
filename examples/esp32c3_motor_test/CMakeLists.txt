# ESP-IDF project: esp32c3_motor_test
# 真机测试：esp32c3 + CAN 收发器 + ZDT_X42S 电机
#
# 本工程位于仓库 examples/ 下，需要把仓库根的 components/zdt_can
# 加进 EXTRA_COMPONENT_DIRS 才能被解析到。
cmake_minimum_required(VERSION 3.16)

# 仓库根 = 本 CMakeLists.txt 往上两级
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(EXTRA_COMPONENT_DIRS "${REPO_ROOT}/components/zdt_can")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32c3_motor_test)
