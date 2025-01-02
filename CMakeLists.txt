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
find_library(RMAN_API_LIB NAMES api_cpp PATHS "${CMAKE_CURRENT_SOURCE_DIR}/lib/RMArm")

if(NOT RMAN_API_LIB)
    message(FATAL_ERROR "api_cpp library not found")
else()
    message(STATUS "RMAN_API_LIB: ${RMAN_API_LIB}")
    # 链接库
    target_link_libraries(${CMAKE_PROJECT_NAME} ${RMAN_API_LIB} pthread)

    # 如果是 UNIX 平台，确保动态库文件可以在运行时被找到（这一步通常不是必须的，但在某些配置下可能需要）
    if(UNIX)
        add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/lib/RMArm/libapi_cpp.so"
                $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>)
    endif()
endif()

add_definitions(-DSPDLOG_FMT_EXTERNAL)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} fmt::fmt spdlog)

message(STATUS "spdlog version: ${spdlog_VERSION}")

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ./include)

# 启用详细输出
set(CMAKE_VERBOSE_MAKEFILE ON)