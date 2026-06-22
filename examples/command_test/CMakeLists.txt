# ESP-IDF project: command_test
# 真机遍历所有可测的 libzdt 命令（黑名单外的 77 条），
# 每条发到电机，检查应答是否含 EE（格式错误）。
cmake_minimum_required(VERSION 3.16)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(EXTRA_COMPONENT_DIRS "${REPO_ROOT}/components/zdt_can")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(command_test)
