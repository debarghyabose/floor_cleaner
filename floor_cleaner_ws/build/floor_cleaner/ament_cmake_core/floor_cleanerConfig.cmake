# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_floor_cleaner_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED floor_cleaner_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(floor_cleaner_FOUND FALSE)
  elseif(NOT floor_cleaner_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(floor_cleaner_FOUND FALSE)
  endif()
  return()
endif()
set(_floor_cleaner_CONFIG_INCLUDED TRUE)

# output package information
if(NOT floor_cleaner_FIND_QUIETLY)
  message(STATUS "Found floor_cleaner: 1.0.0 (${floor_cleaner_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'floor_cleaner' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${floor_cleaner_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(floor_cleaner_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${floor_cleaner_DIR}/${_extra}")
endforeach()
