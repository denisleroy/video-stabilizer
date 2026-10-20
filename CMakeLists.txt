cmake_minimum_required(VERSION 3.20)
project(stabilize VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc video calib3d)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavformat libavcodec libavutil libswscale)

add_library(stabilize_core STATIC
  src/media/video_reader.cpp
  src/media/video_writer.cpp
  src/stab/motion.cpp
  src/stab/trajectory.cpp
  src/stab/warp.cpp
)
target_include_directories(stabilize_core PUBLIC src)
target_link_libraries(stabilize_core PUBLIC ${OpenCV_LIBS} PkgConfig::FFMPEG Threads::Threads)

add_executable(stabilize src/main.cpp)
target_link_libraries(stabilize PRIVATE stabilize_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(stabilize_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(stabilize PRIVATE -Wall -Wextra -Wpedantic)
endif()
