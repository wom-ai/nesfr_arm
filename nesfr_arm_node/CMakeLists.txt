cmake_minimum_required(VERSION 3.5)
project(nesfr_arm_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

option(BUILD_USER_EMAIL "build user email address" "no-one@nowhere")
message("BUILD_USER_EMAIL=${BUILD_USER_EMAIL}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package (Git)
if (GIT_FOUND)
    message("git found: ${GIT_EXECUTABLE} in version ${GIT_VERSION_STRING}")
    # reference: https://stackoverflow.com/questions/51727566/how-to-make-git-commit-hash-available-in-c-code-without-needless-recompiling
    execute_process(
        COMMAND git rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    # Get the latest commit hash
    execute_process(
        COMMAND git rev-parse HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    # reference: https://stackoverflow.com/questions/1404796/how-can-i-get-the-latest-tag-name-in-current-branch-in-git
    # Get the latest tag
    execute_process(
        COMMAND git describe --tags --dirty
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_CURRENT_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    message("GIT_CURRENT_TAG: ${GIT_CURRENT_TAG}")

    execute_process(
        COMMAND git rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE)
endif (GIT_FOUND)

# reference: https://cmake.org/cmake/help/latest/command/cmake_host_system_information.html
cmake_host_system_information(RESULT BUILD_HOSTNAME QUERY HOSTNAME)

add_executable(nesfr_arm_node src/nesfr_arm_node.cpp)
ament_target_dependencies(nesfr_arm_node
    "rclcpp"
    "sensor_msgs"
    )

target_include_directories(nesfr_arm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(
    nesfr_arm_node PUBLIC
    __BUILD_USER_EMAIL__="${BUILD_USER_EMAIL}"
    __BUILD_HOSTNAME__="${BUILD_HOSTNAME}"
    __GIT_BRANCH__="${GIT_BRANCH}"
    __GIT_COMMIT_HASH__="${GIT_COMMIT_HASH}"
    __GIT_CURRENT_TAG__="${GIT_CURRENT_TAG}"
    )

install(TARGETS nesfr_arm_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(nesfr_arm_only_node src/nesfr_arm_only_node.cpp)
ament_target_dependencies(nesfr_arm_only_node
    "rclcpp"
    "geometry_msgs"
    "sensor_msgs"
    "tf2"
    "tf2_ros"
    )

target_include_directories(nesfr_arm_only_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(
    nesfr_arm_only_node PUBLIC
    __BUILD_USER_EMAIL__="${BUILD_USER_EMAIL}"
    __BUILD_HOSTNAME__="${BUILD_HOSTNAME}"
    __GIT_BRANCH__="${GIT_BRANCH}"
    __GIT_COMMIT_HASH__="${GIT_COMMIT_HASH}"
    __GIT_CURRENT_TAG__="${GIT_CURRENT_TAG}"
    )

install(TARGETS nesfr_arm_only_node

  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
