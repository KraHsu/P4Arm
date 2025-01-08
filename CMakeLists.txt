set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")

cmake_minimum_required(VERSION 3.10)
project(RMDemo_Moves)

# 设置 C 标准
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)


set(CMAKE_PROJECT_NAME RM_Arm)

# 设置源文件
file(GLOB_RECURSE SOURCES
    src/*.cpp
    src/Hsu/*.cpp
)

# 添加可执行文件
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})

# 查找 Boost 库
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3 REQUIRED)

# 检查 Boost 是否找到
if (NOT Boost_FOUND)
    message(FATAL_ERROR "Boost library not found. Please install Boost and try again.")
endif()

# 包含 Boost 的头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# 链接 Boost 库
target_link_libraries(${CMAKE_PROJECT_NAME} ${Boost_LIBRARIES} Eigen3::Eigen)

# 添加 Boost 系统库的依赖
target_link_libraries(${CMAKE_PROJECT_NAME} Boost::system)

# 查找库
# 检查系统架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/linux_x86_c++_v1.0.5")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/linux_arm64_c++_v1.0.4")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 查找库
find_library(RMAN_API_LIB NAMES api_cpp PATHS "${LIB_PATH}" REQUIRED)

# 打印找到的库路径
if (RMAN_API_LIB)
    message(STATUS "Found RMAN API library: ${RMAN_API_LIB}")

    message(STATUS "RMAN_API_LIB: ${RMAN_API_LIB}")
else()
    message(FATAL_ERROR "RMAN API library not found in ${LIB_PATH}")
endif()

# 添加定义，告知 spdlog 使用外部 fmt
add_definitions(-DSPDLOG_FMT_EXTERNAL)

# 查找 fmt 和 spdlog 包
set(CMAKE_PREFIX_PATH "/usr/local/fmt-11.0.2" ${CMAKE_PREFIX_PATH})
find_package(fmt 11.0.2 REQUIRED)

if (NOT fmt_VERSION VERSION_EQUAL "11.0.2")
    message(FATAL_ERROR "Found fmt version ${fmt_VERSION}, but required version is 11.0.2")
endif()

message(STATUS "Found fmt version: ${fmt_VERSION}")

set(CMAKE_PREFIX_PATH "/usr/local/spdlog-1.15.0" ${CMAKE_PREFIX_PATH})
find_package(spdlog REQUIRED)

if (NOT spdlog_VERSION VERSION_EQUAL "1.15.0")
    message(FATAL_ERROR "Found spdlog version ${spdlog_VERSION}, but required version is 1.15.0")
endif()

message(STATUS "Found spdlog version: ${spdlog_VERSION}")

# 链接 fmt 和 spdlog 库
target_link_libraries(${CMAKE_PROJECT_NAME} fmt spdlog ${RMAN_API_LIB} pthread)

# 包含项目的头文件目录
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ./include)

# 添加 fmt 和 spdlog 的头文件路径（显式设置避免冲突）
target_include_directories(${CMAKE_PROJECT_NAME} SYSTEM PRIVATE 
    $<TARGET_PROPERTY:fmt::fmt,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:spdlog::spdlog,INTERFACE_INCLUDE_DIRECTORIES>
)

# 强制检查 fmt 和 spdlog 的版本是否兼容
if (spdlog_VERSION VERSION_LESS "1.9.0")
    message(FATAL_ERROR "spdlog version must be at least 1.9.0 to support external fmt")
endif()

option(USE_VISUAL "使用开启可视化功能" OFF)

if(USE_VISUAL)
    message(STATUS "pybind11_DIR is set: " ${pybind11_DIR})

    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    set(PYBIND11_FINDPYTHON ON)
    find_package(pybind11 REQUIRED)

    add_executable(${CMAKE_PROJECT_NAME}_with_visual ${SOURCES})

    target_include_directories(${CMAKE_PROJECT_NAME}_with_visual PRIVATE $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},INCLUDE_DIRECTORIES>)
    target_compile_options(${CMAKE_PROJECT_NAME}_with_visual PRIVATE $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},COMPILE_OPTIONS>)
    target_compile_features(${CMAKE_PROJECT_NAME}_with_visual PRIVATE $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},COMPILE_FEATURES>)
    target_link_libraries(${CMAKE_PROJECT_NAME}_with_visual $<TARGET_PROPERTY:${CMAKE_PROJECT_NAME},LINK_LIBRARIES>)

    target_link_libraries(${CMAKE_PROJECT_NAME}_with_visual pybind11::embed Python3::Python)

    # 为 ${CMAKE_PROJECT_NAME}_with_visual 添加宏定义
    target_compile_definitions(${CMAKE_PROJECT_NAME}_with_visual PRIVATE HSU_FRAME_VISUAL)

    # 添加一个 custom target 用于复制 Python 脚本
    add_custom_target(copy_python ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/py_scripts ${CMAKE_BINARY_DIR}/scripts
        COMMENT "Copying Python scripts to build directory"
    )

    add_dependencies(${CMAKE_PROJECT_NAME}_with_visual copy_python)
endif()

# 启用详细输出
set(CMAKE_VERBOSE_MAKEFILE ON)