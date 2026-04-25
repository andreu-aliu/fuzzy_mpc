cmake_minimum_required(VERSION 3.14)
project(fuzzy_mpc LANGUAGES C CXX)

# ------------------------------------------------------------------------------
# Compiler setup
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release") # TODO: Release

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native -mavx2 -mtune=native -funroll-loops -fno-math-errno -fstrict-aliasing")

add_compile_definitions(EIGEN_MAX_ALIGN_BYTES=64)

# ------------------------------------------------------------------------------
# Dependencies (ROS 2 + system)
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(cat_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(as_lib CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------
# External libraries: BLASFEO + HPIPM (local submodules)
# ------------------------------------------------------------------------------
set(BLASFEO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/blasfeo)
set(HPIPM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/hpipm)

if(NOT EXISTS ${BLASFEO_DIR}/CMakeLists.txt)
  message(FATAL_ERROR "BLASFEO not found in ${BLASFEO_DIR}. Did you clone submodules? Run:
    git submodule update --init --recursive")
endif()

if(NOT EXISTS ${HPIPM_DIR}/CMakeLists.txt)
  message(FATAL_ERROR "HPIPM not found in ${HPIPM_DIR}. Did you clone submodules? Run:
    git submodule update --init --recursive")
endif()

# Build them locally
message(STATUS "Building local BLASFEO and HPIPM from external/")

add_subdirectory(${BLASFEO_DIR} EXCLUDE_FROM_ALL)
add_subdirectory(${HPIPM_DIR} EXCLUDE_FROM_ALL)

include_directories(
  ${BLASFEO_DIR}/include
  ${HPIPM_DIR}/include
)

set(BLASFEO_LIBRARIES blasfeo)
set(HPIPM_LIBRARIES hpipm)

# ------------------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
  as_lib::as_lib
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  visualization_msgs
  nav_msgs
  cat_msgs
  tf2
  ament_index_cpp
)

target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)

target_link_libraries(${PROJECT_NAME}
  ${HPIPM_LIBRARIES}
  ${BLASFEO_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (YAML)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install model files (YAML)
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)


# ------------------------------------------------------------------------------
# Ament exports
# ------------------------------------------------------------------------------
ament_export_include_directories(include)
ament_package()
