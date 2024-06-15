# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rc_decision_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rc_decision_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rc_decision_FOUND FALSE)
  elseif(NOT rc_decision_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rc_decision_FOUND FALSE)
  endif()
  return()
endif()
set(_rc_decision_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rc_decision_FIND_QUIETLY)
  message(STATUS "Found rc_decision: 0.0.0 (${rc_decision_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rc_decision' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rc_decision_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rc_decision_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rc_decision_DIR}/${_extra}")
endforeach()
