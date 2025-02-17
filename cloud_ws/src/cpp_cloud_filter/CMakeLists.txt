cmake_minimum_required(VERSION 3.8)
project(cpp_cloud_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

# using ament_auto to automatically handle dependencies based on the package.xml file
ament_auto_find_build_dependencies()

ament_auto_add_executable(filter src/filter.cpp)
target_compile_features(filter PUBLIC cxx_std_17)  # Require C99 and C++17

install(TARGETS filter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
