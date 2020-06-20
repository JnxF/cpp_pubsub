cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

set(CMAKE_BUILD_TYPE "Debug")
include(${CMAKE_SOURCE_DIR}/cmake/CodeCoverage.cmake)
APPEND_COVERAGE_COMPILER_FLAGS()
set(COVERAGE_LCOV_EXCLUDES 'launch/*' 'inputs/*')
SETUP_TARGET_FOR_COVERAGE_LCOV(NAME "listener_test_project" EXECUTABLE "./listener")



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()



# Talker
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

# Listener
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})

# Inject
add_library(injector SHARED src/injector.cpp)
ament_target_dependencies(injector rclcpp std_msgs)
set_property(TARGET injector PROPERTY POSITION_INDEPENDENT_CODE ON)
install(TARGETS
  injector
  DESTINATION lib/${PROJECT_NAME})

# Fuzztarget
#add_executable(cpp_fuzztarget src/cpp_fuzztarget.cpp)
#ament_target_dependencies(cpp_fuzztarget rclcpp std_msgs)

#install(TARGETS
#  cpp_fuzztarget
#  DESTINATION lib/${PROJECT_NAME})

# Copy input folder to test with AFL
install(DIRECTORY inputs
	DESTINATION lib/${PROJECT_NAME}
)


ament_package()
