# CMakeList.txt: PelvisCollisionDetection 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)
find_package(VTK REQUIRED)
# 将源代码添加到此项目的可执行文件。
include(${VTK_USE_FILE})
# 将源代码添加到此项目的可执行文件。
add_executable (PelvisCollisionDetection "PelvisCollisionDetection.cpp" 
"PelvisCollisionDetection.h" 
"FileToMatrix.h" 
"FileToMatrix.cpp" 
"CollisionCallback.h" 
"CollisionCallback.cpp" 
"collisionDetect.h"
"collisionDetect.cpp")
target_link_libraries(PelvisCollisionDetection ${VTK_LIBRARIES})

vtk_module_autoinit(
  TARGETS PelvisCollisionDetection
  MODULES ${VTK_LIBRARIES}
  )
# TODO: 如有需要，请添加测试并安装目标。
