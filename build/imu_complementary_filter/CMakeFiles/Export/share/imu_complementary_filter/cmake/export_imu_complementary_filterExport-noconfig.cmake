#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "imu_complementary_filter::complementary_filter" for configuration ""
set_property(TARGET imu_complementary_filter::complementary_filter APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(imu_complementary_filter::complementary_filter PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcomplementary_filter.so"
  IMPORTED_SONAME_NOCONFIG "libcomplementary_filter.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS imu_complementary_filter::complementary_filter )
list(APPEND _IMPORT_CHECK_FILES_FOR_imu_complementary_filter::complementary_filter "${_IMPORT_PREFIX}/lib/libcomplementary_filter.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
