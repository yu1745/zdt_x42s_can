cmake_minimum_required(VERSION 3.16)

# 仓库根即组件根（zdt_can 组件直接在仓库根），用 EXTRA_COMPONENT_DIRS 指向它
# examples/command_test 往上两级才到仓库根：command_test -> examples -> zdt_x42s_can
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(EXTRA_COMPONENT_DIRS "${REPO_ROOT}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(command_test)
