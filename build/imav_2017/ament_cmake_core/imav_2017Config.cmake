# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_imav_2017_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED imav_2017_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(imav_2017_FOUND FALSE)
  elseif(NOT imav_2017_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(imav_2017_FOUND FALSE)
  endif()
  return()
endif()
set(_imav_2017_CONFIG_INCLUDED TRUE)

# output package information
if(NOT imav_2017_FIND_QUIETLY)
  message(STATUS "Found imav_2017: 0.0.1 (${imav_2017_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'imav_2017' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${imav_2017_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(imav_2017_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${imav_2017_DIR}/${_extra}")
endforeach()
