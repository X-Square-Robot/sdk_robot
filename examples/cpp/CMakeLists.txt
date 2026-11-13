cmake_minimum_required(VERSION 3.16)
project(sdk_client_examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Expected extracted artifact layout:
#   <root>/include
#   <root>/lib/libsdk_client_lib.so
#   <root>/examples/cpp/CMakeLists.txt
get_filename_component(_default_sdk_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(X2ROBOT_SDK_ROOT "${_default_sdk_root}" CACHE PATH
    "Extracted sdk-client artifact root")

set(_sdk_include "${X2ROBOT_SDK_ROOT}/include")
find_library(X2ROBOT_SDK_CLIENT_LIBRARY
  NAMES sdk_client_lib
  PATHS "${X2ROBOT_SDK_ROOT}/lib"
  NO_DEFAULT_PATH)
find_library(X2ROBOT_PROTOBUF_LIBRARY
  NAMES protobuf
  PATHS "${X2ROBOT_SDK_ROOT}/lib"
  NO_DEFAULT_PATH)
find_library(X2ROBOT_GRPCPP_LIBRARY
  NAMES grpc++
  PATHS "${X2ROBOT_SDK_ROOT}/lib"
  NO_DEFAULT_PATH)
find_library(X2ROBOT_GRPC_LIBRARY
  NAMES grpc
  PATHS "${X2ROBOT_SDK_ROOT}/lib"
  NO_DEFAULT_PATH)
find_library(X2ROBOT_GPR_LIBRARY
  NAMES gpr
  PATHS "${X2ROBOT_SDK_ROOT}/lib"
  NO_DEFAULT_PATH)

if(NOT EXISTS "${_sdk_include}/x2robot/sdk/robot.hpp")
  message(FATAL_ERROR
    "SDK headers not found under ${_sdk_include}; set -DX2ROBOT_SDK_ROOT=<artifact-root>")
endif()
if(NOT EXISTS "${_sdk_include}/v2/common/result.pb.h")
  message(FATAL_ERROR
    "SDK v2 protocol headers not found under ${_sdk_include}/v2")
endif()
if(NOT X2ROBOT_SDK_CLIENT_LIBRARY)
  message(FATAL_ERROR
    "libsdk_client_lib.so not found under ${X2ROBOT_SDK_ROOT}/lib")
endif()
if(NOT X2ROBOT_PROTOBUF_LIBRARY)
  message(FATAL_ERROR
    "libprotobuf.so not found under ${X2ROBOT_SDK_ROOT}/lib")
endif()
if(NOT X2ROBOT_GRPCPP_LIBRARY OR NOT X2ROBOT_GRPC_LIBRARY OR NOT X2ROBOT_GPR_LIBRARY)
  message(FATAL_ERROR
    "bundled gRPC libraries not found under ${X2ROBOT_SDK_ROOT}/lib")
endif()

add_library(x2robot_sdk_client SHARED IMPORTED)
set_target_properties(x2robot_sdk_client PROPERTIES
  IMPORTED_LOCATION "${X2ROBOT_SDK_CLIENT_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${_sdk_include}")

# Public SDK methods exchange generated protobuf messages.  Examples that
# inspect those messages therefore reference libprotobuf symbols directly;
# relying on libsdk_client_lib.so's DT_NEEDED entry is not sufficient with
# modern linkers using --no-copy-dt-needed-entries.
add_library(x2robot_sdk_protobuf SHARED IMPORTED)
set_target_properties(x2robot_sdk_protobuf PROPERTIES
  IMPORTED_LOCATION "${X2ROBOT_PROTOBUF_LIBRARY}")

add_library(x2robot_sdk_grpcpp SHARED IMPORTED)
set_target_properties(x2robot_sdk_grpcpp PROPERTIES
  IMPORTED_LOCATION "${X2ROBOT_GRPCPP_LIBRARY}")
add_library(x2robot_sdk_grpc SHARED IMPORTED)
set_target_properties(x2robot_sdk_grpc PROPERTIES
  IMPORTED_LOCATION "${X2ROBOT_GRPC_LIBRARY}")
add_library(x2robot_sdk_gpr SHARED IMPORTED)
set_target_properties(x2robot_sdk_gpr PROPERTIES
  IMPORTED_LOCATION "${X2ROBOT_GPR_LIBRARY}")

find_package(Threads REQUIRED)

set(SDK_CPP_EXAMPLES_DIR "${CMAKE_CURRENT_LIST_DIR}")
# Clear the directory variable explicitly.  An earlier configure may have
# cached a target that was removed from the public example list; `set(VAR)`
# would expose that cache value again on the next configure.
set(SDK_CPP_EXAMPLE_TARGETS "")

function(sdk_add_example example source)
  set(source_path "${SDK_CPP_EXAMPLES_DIR}/${source}")
  if(NOT EXISTS "${source_path}")
    message(FATAL_ERROR "Missing C++ example source: ${source}")
  endif()
  add_executable(${example} "${source_path}")
  target_sources(${example} PRIVATE "${SDK_CPP_EXAMPLES_DIR}/interrupt.cpp")
  target_include_directories(${example} PRIVATE "${SDK_CPP_EXAMPLES_DIR}")
  target_link_libraries(${example} PRIVATE
    x2robot_sdk_client x2robot_sdk_protobuf
    x2robot_sdk_grpcpp x2robot_sdk_grpc x2robot_sdk_gpr
    Threads::Threads ${CMAKE_DL_LIBS} rt)
  set_target_properties(${example} PROPERTIES
    BUILD_RPATH "${X2ROBOT_SDK_ROOT}/lib"
    INSTALL_RPATH "$ORIGIN/../lib")
  list(APPEND SDK_CPP_EXAMPLE_TARGETS "${example}")
  set(SDK_CPP_EXAMPLE_TARGETS "${SDK_CPP_EXAMPLE_TARGETS}" PARENT_SCOPE)
endfunction()

# The legacy Python v1 tree remains the behavioral specification for the
# existing model examples. Quanta X3 is v2-only; its arm, waist and gripper
# motion entrypoints are paired separately below. Shared
# head/chassis/system/audio/navigation examples use the v2 root directory.
set(SDK_PYTHON_V1_DIR "${CMAKE_CURRENT_LIST_DIR}/../python/v1")
set(SDK_PYTHON_V1_CPP_PAIRS
  "audio.py=audio.cpp"
  "camera.py=camera.cpp"
  "chassis_control.py=chassis_control.cpp"
  "check_connect.py=check_connect.cpp"
  "data_collection/collection_config.py=data_collection/collection_config.hpp"
  "data_collection/data_collector.py=data_collection/data_collector.cpp"
  "data_collection_example.py=data_collection_example.cpp"
  "data_replay_example.py=data_replay_example.cpp"
  "depth_points.py=depth_points.cpp"
  "desktop/arm_control.py=desktop/arm_control.cpp"
  "desktop/gripper_control.py=desktop/gripper_control.cpp"
  "desktop/master_arm_control.py=desktop/master_arm_control.cpp"
  "head_control.py=head_control.cpp"
  "imu.py=imu.cpp"
  "navigation.py=navigation.cpp"
  "quanta_x1/align_master_slave_demo.py=quanta_x1/align_master_slave_demo.cpp"
  "quanta_x1/arm_control.py=quanta_x1/arm_control.cpp"
  "quanta_x1/gripper_control.py=quanta_x1/gripper_control.cpp"
  "quanta_x1/lift_control.py=quanta_x1/lift_control.cpp"
  "quanta_x1/master_arm_control.py=quanta_x1/master_arm_control.cpp"
  "quanta_x2/arm_control.py=quanta_x2/arm_control.cpp"
  "quanta_x2/end_pose_replay_new.py=quanta_x2/end_pose_replay_new.cpp"
  "quanta_x2/gripper_control.py=quanta_x2/gripper_control.cpp"
  "quanta_x2/waist_control.py=quanta_x2/waist_control.cpp"
  "radar.py=radar.cpp"
  "robot_control.py=robot_control.cpp"
  "robot_status.py=robot_status.cpp"
  "system.py=system.cpp"
  "tf.py=tf.cpp"
  "tof.py=tof.cpp"
  "ultrasonic.py=ultrasonic.cpp")
foreach(pair IN LISTS SDK_PYTHON_V1_CPP_PAIRS)
  string(REPLACE "=" ";" pair_parts "${pair}")
  list(GET pair_parts 0 python_relative)
  list(GET pair_parts 1 cpp_relative)
  if(NOT EXISTS "${SDK_PYTHON_V1_DIR}/${python_relative}")
    message(FATAL_ERROR "Missing Python v1 example: ${python_relative}")
  endif()
  if(NOT EXISTS "${SDK_CPP_EXAMPLES_DIR}/${cpp_relative}")
    message(FATAL_ERROR "Missing C++ counterpart for Python v1 example: ${python_relative}")
  endif()
endforeach()

set(SDK_PYTHON_V2_DIR "${CMAKE_CURRENT_LIST_DIR}/../python/v2")
set(SDK_PYTHON_V2_QUANTA_X3_CPP_PAIRS
  "quanta_x3/arm_control.py=quanta_x3/arm_control.cpp"
  "quanta_x3/gripper_control.py=quanta_x3/gripper_control.cpp"
  "quanta_x3/lidar_points.py=quanta_x3/lidar_points.cpp"
  "quanta_x3/waist_control.py=quanta_x3/waist_control.cpp")
foreach(pair IN LISTS SDK_PYTHON_V2_QUANTA_X3_CPP_PAIRS)
  string(REPLACE "=" ";" pair_parts "${pair}")
  list(GET pair_parts 0 python_relative)
  list(GET pair_parts 1 cpp_relative)
  if(NOT EXISTS "${SDK_PYTHON_V2_DIR}/${python_relative}")
    message(FATAL_ERROR "Missing Python v2 quanta_x3 example: ${python_relative}")
  endif()
  if(NOT EXISTS "${SDK_CPP_EXAMPLES_DIR}/${cpp_relative}")
    message(FATAL_ERROR "Missing C++ quanta_x3 counterpart: ${cpp_relative}")
  endif()
endforeach()

include("${SDK_CPP_EXAMPLES_DIR}/examples.cmake")

install(TARGETS ${SDK_CPP_EXAMPLE_TARGETS} RUNTIME DESTINATION bin)

set(SDK_CPP_EXAMPLE_TARGETS "${SDK_CPP_EXAMPLE_TARGETS}" CACHE INTERNAL
  "All standalone SDK C++ example targets")

option(SDK_EXAMPLES_BUILD_TESTS "Build loopback C++ example regression tests" OFF)
if(SDK_EXAMPLES_BUILD_TESTS)
  enable_testing()
  add_executable(arm_end_pose_test tests/arm_end_pose.cpp interrupt.cpp)
  target_include_directories(arm_end_pose_test PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
  target_link_libraries(arm_end_pose_test PRIVATE
    x2robot_sdk_client x2robot_sdk_protobuf
    x2robot_sdk_grpcpp x2robot_sdk_grpc x2robot_sdk_gpr
    Threads::Threads ${CMAKE_DL_LIBS} rt)
  add_test(NAME arm_end_pose_regression COMMAND arm_end_pose_test)
  set_tests_properties(arm_end_pose_regression PROPERTIES TIMEOUT 20)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(tf_test_server tests/tf_server.cpp)
  target_link_libraries(tf_test_server PRIVATE
    x2robot_sdk_client x2robot_sdk_protobuf
    x2robot_sdk_grpcpp x2robot_sdk_grpc x2robot_sdk_gpr
    Threads::Threads ${CMAKE_DL_LIBS} rt)
  add_test(NAME tf_example_regression COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_tf.py"
    "$<TARGET_FILE:tf>" "$<TARGET_FILE:tf_test_server>")
  set_tests_properties(tf_example_regression PROPERTIES TIMEOUT 90)
  add_executable(body_stream_test_server tests/body_stream_server.cpp)
  target_link_libraries(body_stream_test_server PRIVATE
    x2robot_sdk_client x2robot_sdk_protobuf
    x2robot_sdk_grpcpp x2robot_sdk_grpc x2robot_sdk_gpr
    Threads::Threads ${CMAKE_DL_LIBS} rt)
  add_test(NAME body_stream_example_regression COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_body_streams.py"
    "$<TARGET_FILE:body_stream_test_server>" "$<TARGET_FILE_DIR:desktop_arm_control>")
  set_tests_properties(body_stream_example_regression PROPERTIES TIMEOUT 90)
  add_test(NAME collection_input_regression COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_collection_input.py"
    "$<TARGET_FILE:data_collection_example>" "$<TARGET_FILE:body_stream_test_server>")
  set_tests_properties(collection_input_regression PROPERTIES TIMEOUT 90)
  add_executable(sensor_test_server tests/sensor_server.cpp)
  target_link_libraries(sensor_test_server PRIVATE
    x2robot_sdk_client x2robot_sdk_protobuf
    x2robot_sdk_grpcpp x2robot_sdk_grpc x2robot_sdk_gpr
    Threads::Threads ${CMAKE_DL_LIBS} rt)
  add_test(NAME sensor_example_regression COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_sensors.py"
    "$<TARGET_FILE:sensor_test_server>" "$<TARGET_FILE_DIR:depth_points>")
  set_tests_properties(sensor_example_regression PROPERTIES TIMEOUT 90)
  add_test(NAME audio_interrupt_regression COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_audio_interrupt.py"
    "$<TARGET_FILE:audio>" "$<TARGET_FILE:sensor_test_server>")
  set_tests_properties(audio_interrupt_regression PROPERTIES TIMEOUT 20)
  add_executable(homing_test_server tests/homing_server.cpp)
  target_link_libraries(homing_test_server PRIVATE
    x2robot_sdk_client x2robot_sdk_protobuf
    x2robot_sdk_grpcpp x2robot_sdk_grpc x2robot_sdk_gpr
    Threads::Threads ${CMAKE_DL_LIBS} rt)
  add_test(NAME homing_example_regression COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_homing.py"
    "$<TARGET_FILE:robot_control>" "$<TARGET_FILE:homing_test_server>")
  set_tests_properties(homing_example_regression PROPERTIES TIMEOUT 45)
endif()
