
cmake_minimum_required(VERSION 3.1...3.20)
project(MyProject)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找到CGAL
# 添加局部CGAL库的路径
set(CGAL_DIR "${CMAKE_SOURCE_DIR}/lib/CGAL/lib/cmake/CGAL")
find_package(CGAL REQUIRED )
message(STATUS "CGAL_UDIR is set to ${CGAL_DIR}")
message(STATUS "CGAL is set to ${CGAL_INCLUDE_DIRS}")

find_package(Threads REQUIRED)

find_package(TBB QUIET)
include(CGAL_TBB_support)

find_package(OpenCV 4.0 REQUIRED)
message(STATUS "OpenCV is set to ${OpenCV_INCLUDE_DIRS}")

include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories(
${PROJECT_SOURCE_DIR}
${CGAL_DIR}/lib/CGAL/include/CGAL
${PROJECT_SOURCE_DIR}/lib
${PROJECT_SOURCE_DIR}/lib/RTFMT
${EIGEN3_INCLUDE_DIR}
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples)

# 该版本RT-FMT*代价函数采用双键值表示，所以被阻隔区域的树依然可以被维护。
# 但由于依旧采用类启发式获取路径，因此并不影响性能，与运行效果

add_executable(test
test/test.cpp
lib/RTFMT/RTFMT.cpp
)

target_link_libraries(test PUBLIC 
CGAL::CGAL 
CGAL::TBB_support
${OpenCV_LIBS}
pthread
)

# 四个动态环境的实施规划测试
add_executable(testD
test/testD.cpp
lib/RTFMT/RTFMT.cpp
)

target_link_libraries(testD PUBLIC 
CGAL::CGAL 
CGAL::TBB_support
${OpenCV_LIBS}
pthread
)


