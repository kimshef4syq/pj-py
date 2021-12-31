include(CMakeFindDependencyMacro)
set(RCI_DIR /home/rvbust/Rvbust/Install/RVBUST/RCI/CMake)
find_dependency(RCI)

get_filename_component(_prefix "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
include("${CMAKE_CURRENT_LIST_DIR}/RPS-targets.cmake")
file(GLOB_RECURSE _external_libs "${_prefix}/Lib/Externals/*.so")
target_link_libraries(RPS::RPS
                      INTERFACE ${_external_libs} RCI::RCI)
if(NOT RPS_FIND_QUIETLY)
  message(STATUS "found lib RPS")
endif()
