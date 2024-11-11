# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_race_it_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED race_it_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(race_it_FOUND FALSE)
  elseif(NOT race_it_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(race_it_FOUND FALSE)
  endif()
  return()
endif()
set(_race_it_CONFIG_INCLUDED TRUE)

# output package information
if(NOT race_it_FIND_QUIETLY)
  message(STATUS "Found race_it: 0.0.0 (${race_it_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'race_it' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${race_it_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(race_it_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${race_it_DIR}/${_extra}")
endforeach()
