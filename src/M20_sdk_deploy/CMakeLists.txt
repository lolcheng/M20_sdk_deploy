cmake_minimum_required(VERSION 3.10)

project(m20_sdk_deploy)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# 警告忽略和编译选项
add_definitions(-w)
add_compile_options(-fPIC)

option(M20_JOINT_BENCH_ONLY "Build only the M20 Pro manual joint bench" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drdds REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(M20_SIM_DIR interface/robot/simulation)
set(M20_PRO_DIR M20_description/M20_Pro)
set(M20_PRO_URDF "${CMAKE_CURRENT_SOURCE_DIR}/${M20_PRO_DIR}/urdf/M20.urdf")
set(M20_URDF_READER "${CMAKE_CURRENT_SOURCE_DIR}/${M20_SIM_DIR}/m20_pro_urdf.py")
set(M20_LIMITS_HEADER "${CMAKE_CURRENT_BINARY_DIR}/m20_pro_urdf_limits.hpp")
add_custom_command(OUTPUT "${M20_LIMITS_HEADER}"
  COMMAND "${Python3_EXECUTABLE}" "${M20_URDF_READER}" "${M20_PRO_URDF}" "${M20_LIMITS_HEADER}"
  DEPENDS "${M20_URDF_READER}" "${M20_PRO_URDF}"
  VERBATIM)
add_executable(m20_joint_bench_node tools/m20_joint_bench.cpp "${M20_LIMITS_HEADER}")
target_include_directories(m20_joint_bench_node PRIVATE include/types third_party/eigen "${CMAKE_CURRENT_BINARY_DIR}")
ament_target_dependencies(m20_joint_bench_node rclcpp drdds sensor_msgs rmw)
install(TARGETS m20_joint_bench_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/m20_joint_bench_launcher.py DESTINATION lib/${PROJECT_NAME} RENAME m20_joint_bench)

set(M20_MODEL_DIR M20_description/m20_mjcf)
install(FILES ${M20_SIM_DIR}/mujoco_simulation_ros2.py ${M20_SIM_DIR}/m20_sim_config.py "${M20_URDF_READER}"
  DESTINATION share/${PROJECT_NAME}/${M20_SIM_DIR})
install(FILES ${M20_MODEL_DIR}/mjcf/M20_joint_demo.xml ${M20_MODEL_DIR}/mjcf/M20.xml
  DESTINATION share/${PROJECT_NAME}/${M20_MODEL_DIR}/mjcf)
install(DIRECTORY ${M20_MODEL_DIR}/meshes/
  DESTINATION share/${PROJECT_NAME}/${M20_MODEL_DIR}/meshes FILES_MATCHING PATTERN "*.STL")
install(FILES "${M20_PRO_URDF}" DESTINATION share/${PROJECT_NAME}/${M20_PRO_DIR}/urdf)
install(DIRECTORY ${M20_PRO_DIR}/mjcf/ DESTINATION share/${PROJECT_NAME}/${M20_PRO_DIR}/mjcf
  FILES_MATCHING PATTERN "*.xml" PATTERN "*.STL")

if(M20_JOINT_BENCH_ONLY)
  ament_package()
  return()
endif()

# 配置选项
set(BUILD_PLATFORM "x86" CACHE STRING "Target platform (x86 or arm)")

if(BUILD_PLATFORM STREQUAL "arm")
  set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
endif()

# 查找依赖包
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

include_directories(
  include/types
  include/utils
  interface/robot
  interface/user_command
  interface/network
  state_machine
  run_policy
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/include
)

# 子目录
add_subdirectory(interface)

# 运行时路径设置
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


set(CMAKE_INSTALL_RPATH 
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/
  ../lib/onnxruntime/
)

file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")

add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC})

add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${THIRD_PARTY}/onnxruntime//${BUILD_PLATFORM}/lib/libonnxruntime.so")

ament_target_dependencies(rl_deploy
  rclcpp
  drdds
  std_msgs
  geometry_msgs
)
if(BUILD_PLATFORM STREQUAL "arm")
  target_link_libraries(rl_deploy
    interface
    onnxruntime
    -lpthread
    -lm
    -lrt
    -ldl
    -lstdc++
  )
else()
  target_link_libraries(rl_deploy
    interface
    onnxruntime
    -lpthread
    -lm
    -lrt
    -ldl
    -lstdc++
  )
endif()


install(TARGETS rl_deploy
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
