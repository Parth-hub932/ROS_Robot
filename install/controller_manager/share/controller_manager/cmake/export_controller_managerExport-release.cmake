#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "controller_manager::controller_manager" for configuration "Release"
set_property(TARGET controller_manager::controller_manager APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(controller_manager::controller_manager PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcontroller_manager.so"
  IMPORTED_SONAME_RELEASE "libcontroller_manager.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS controller_manager::controller_manager )
list(APPEND _IMPORT_CHECK_FILES_FOR_controller_manager::controller_manager "${_IMPORT_PREFIX}/lib/libcontroller_manager.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
