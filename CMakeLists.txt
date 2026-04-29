cmake_minimum_required(VERSION 3.14)
project(g1_onnx_rl)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics")
find_package(Boost)
find_package(nlohmann_json 3.7.3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(unitree_sdk2 REQUIRED)

set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-x64-1.22.0)
# set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-aarch64-1.22.0)

include_directories(
    include
    ${ONNXRUNTIME_ROOT}/include
    ${ONNXRUNTIME_ROOT}/include/onnxruntime
    ${ONNXRUNTIME_ROOT}/include/onnxruntime/core/session
    ${Boost_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${OpenSSL_INCLUDE_DIR}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread")

link_directories(${ONNXRUNTIME_ROOT}/lib)

file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

add_executable(wbc_fsm src/main.cpp ${SRC_LIST})

target_compile_definitions(wbc_fsm PRIVATE 
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(wbc_fsm PRIVATE
    ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so.1.22.0
    unitree_sdk2
    ${CUDA_LIBRARIES}
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    ${Boost_LIBRARIES}
)