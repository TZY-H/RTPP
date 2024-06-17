
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
${PROJECT_SOURCE_DIR}/lib/CDRT
${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples)

# 请先运行approx_work.py，python3.7.9, opencv-python==3.4.9

# 一个简单的键鼠交互的测试程序
add_executable(main 
test/main.cpp
lib/CDRT/CDRT.cpp
lib/CDRT/CDRTmapcd.cpp
lib/CDRT/CDRTcommon.cpp
)
target_link_libraries(main PUBLIC 
CGAL::CGAL 
CGAL::TBB_support
${OpenCV_LIBS}
pthread
)

# 四个静态环境的实验
add_executable(testS
test/testS.cpp
lib/CDRT/CDRT.cpp
lib/CDRT/CDRTmapcd.cpp
lib/CDRT/CDRTcommon.cpp
)
target_link_libraries(testS PUBLIC 
CGAL::CGAL 
CGAL::TBB_support
${OpenCV_LIBS}
pthread
)

# 四个动态环境的实验
add_executable(testD 
test/testD.cpp
lib/CDRT/CDRT.cpp
lib/CDRT/CDRTmapcd.cpp
lib/CDRT/CDRTcommon.cpp
)
target_link_libraries(testD PUBLIC 
CGAL::CGAL 
CGAL::TBB_support
${OpenCV_LIBS}
pthread
)

# 户外机械狗的仿真测试程序
add_executable(dogtest 
test/dogtest.cpp
lib/CDRT/CDRT.cpp
lib/CDRT/CDRTmapcd.cpp
lib/CDRT/CDRTcommon.cpp
)
target_link_libraries(dogtest PUBLIC 
CGAL::CGAL 
CGAL::TBB_support
${OpenCV_LIBS}
pthread
)

