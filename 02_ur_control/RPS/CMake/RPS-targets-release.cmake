#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "RPS::RPS" for configuration "Release"
set_property(TARGET RPS::RPS APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(RPS::RPS PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/Lib/libRPS.so.0.1.1"
  IMPORTED_SONAME_RELEASE "libRPS.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS RPS::RPS )
list(APPEND _IMPORT_CHECK_FILES_FOR_RPS::RPS "${_IMPORT_PREFIX}/Lib/libRPS.so.0.1.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
