cmake_minimum_required(VERSION 3.16)
project(pi05_franka_bridge LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(PI05_WITH_FRANKA "Build the real libfranka backend" ON)
option(PI05_FETCH_DEPS "Fetch pinned Ruckig/JSON dependencies" OFF)

if(PI05_FETCH_DEPS)
  include(FetchContent)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
  set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(ruckig URL https://codeload.github.com/pantor/ruckig/tar.gz/refs/tags/v0.15.3
    URL_HASH SHA256=949c8b0b3227cf3f71a9750126b2952ba3a7af43e3be85505a853ebd17671166)
  FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
  FetchContent_MakeAvailable(ruckig json)
else()
  find_package(ruckig 0.15 REQUIRED CONFIG)
  find_package(nlohmann_json 3.11 REQUIRED CONFIG)
endif()
find_package(Threads REQUIRED)
add_executable(pi05_franka_bridge bridge.cpp)
target_link_libraries(pi05_franka_bridge PRIVATE ruckig::ruckig nlohmann_json::nlohmann_json Threads::Threads)
target_compile_options(pi05_franka_bridge PRIVATE -Wall -Wextra -Wpedantic)
if(PI05_WITH_FRANKA)
  # Franka Emika Panda ships with libfranka 0.9.x; FR3 ships with 0.15.x.
  # The bridge uses only APIs that exist in both (control/JointPositions,
  # setCollisionBehavior 8-array, setJointImpedance, setLoad, readOnce,
  # RealtimeConfig::kEnforce, ControllerMode::kJointImpedance,
  # kMaxCutoffFrequency, q_d/dq_d/ddq_d in RobotState).
  # Do not set a minimum version here; let the installed system package
  # satisfy the request regardless of whether it is 0.9 or 0.15.
  # Official Robotiq SDK 1.2.0, pinned in vendor/robotiq_grippers.
  set(GRIPPERS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GRIPPERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GRIPPERS_BUILD_FAKE OFF CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/robotiq_grippers/sdk_cpp
                   ${CMAKE_CURRENT_BINARY_DIR}/robotiq_sdk)
  target_sources(pi05_franka_bridge PRIVATE robotiq_gripper.cpp)
  target_link_libraries(pi05_franka_bridge PRIVATE Robotiq::grippers)
  find_package(Franka REQUIRED)
  target_link_libraries(pi05_franka_bridge PRIVATE Franka::Franka)
  target_compile_definitions(pi05_franka_bridge PRIVATE PI05_WITH_FRANKA=1)
  # Reuse the real bridge's exact Robotiq driver without opening a Panda connection.
  add_executable(pi05_gripper_tool gripper_tool.cpp robotiq_gripper.cpp)
  target_link_libraries(pi05_gripper_tool PRIVATE Robotiq::grippers nlohmann_json::nlohmann_json Threads::Threads)
  target_compile_options(pi05_gripper_tool PRIVATE -Wall -Wextra -Wpedantic)
  add_executable(gripper_activation_probe gripper_activation_probe.cpp robotiq_gripper.cpp)
  target_link_libraries(gripper_activation_probe PRIVATE Robotiq::grippers Threads::Threads)
endif()
enable_testing()
add_executable(bridge_core_test core_test.cpp)
target_link_libraries(bridge_core_test PRIVATE ruckig::ruckig nlohmann_json::nlohmann_json Threads::Threads)
add_test(NAME bridge_core_test COMMAND bridge_core_test)

if(PI05_WITH_FRANKA)
  add_executable(fault_history_test fault_history_test.cpp)
  target_link_libraries(fault_history_test PRIVATE Franka::Franka nlohmann_json::nlohmann_json)
  add_test(NAME fault_history_test COMMAND fault_history_test)
endif()
