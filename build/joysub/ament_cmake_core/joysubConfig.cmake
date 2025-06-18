# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_joysub_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED joysub_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(joysub_FOUND FALSE)
  elseif(NOT joysub_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(joysub_FOUND FALSE)
  endif()
  return()
endif()
set(_joysub_CONFIG_INCLUDED TRUE)

# output package information
if(NOT joysub_FIND_QUIETLY)
  message(STATUS "Found joysub: 0.0.0 (${joysub_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'joysub' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT joysub_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(joysub_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${joysub_DIR}/${_extra}")
endforeach()
