#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "controller_interface::controller_interface" for configuration "Release"
set_property(TARGET controller_interface::controller_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(controller_interface::controller_interface PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcontroller_interface.so"
  IMPORTED_SONAME_RELEASE "libcontroller_interface.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS controller_interface::controller_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_controller_interface::controller_interface "${_IMPORT_PREFIX}/lib/libcontroller_interface.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
