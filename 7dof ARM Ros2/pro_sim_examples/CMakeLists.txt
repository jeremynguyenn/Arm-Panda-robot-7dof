cmake_minimum_required(VERSION 3.5)
project(pro_sim_examples)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing and linting
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Install C++ examples
set(EXAMPLES_CPP_DIR examples/cpp)
# Example 0 - Follow target
set(EXECUTABLE_0 ex_follow_target)
# Example 1 - Move Object
set(EXECUTABLE_1 ex_move_object)
add_executable(${EXECUTABLE_0} ${EXAMPLES_CPP_DIR}/${EXECUTABLE_0}.cpp)
add_executable(${EXECUTABLE_1} ${EXAMPLES_CPP_DIR}/${EXECUTABLE_1}.cpp)
ament_target_dependencies(${EXECUTABLE_0}
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
)
ament_target_dependencies(${EXECUTABLE_1}
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
)
install(TARGETS
    ${EXECUTABLE_0}
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
    ${EXECUTABLE_1}
    DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch rviz worlds DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
