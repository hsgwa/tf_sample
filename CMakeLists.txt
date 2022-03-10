cmake_minimum_required(VERSION 3.8)
project(tf_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(tf2_broadcaster src/tf2_broadcaster.cpp)
ament_auto_add_executable(tf2_listener src/tf2_listener.cpp)
ament_auto_add_executable(tf2_chain src/tf2_chain.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
