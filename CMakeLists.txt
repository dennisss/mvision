cmake_minimum_required(VERSION 2.8)
project(MVision)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ZLIB REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)



add_library(vio
	src/msckf.cpp
	src/msckf_imgproc.cpp
	src/utils.cpp
	src/projection.cpp
	src/stat.cpp
	src/fileio.cpp
)
set_target_properties(vio PROPERTIES COMPILE_FLAGS "-std=c++11 -w -g -O3") #  -g
target_link_libraries(vio ${OpenCV_LIBS} ${ZLIB_LIBRARIES})


add_executable(mvision
	src/main.cpp
	src/MadgwickAHRS.cpp
	src/testing.cpp
)

set_target_properties(mvision PROPERTIES COMPILE_FLAGS "-std=c++11 -w -g -O3") #  -g
target_link_libraries(mvision ${OpenCV_LIBS} vio)
