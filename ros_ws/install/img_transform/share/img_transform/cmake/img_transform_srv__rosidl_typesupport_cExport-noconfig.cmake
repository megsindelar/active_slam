#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "img_transform::img_transform_srv__rosidl_typesupport_c" for configuration ""
set_property(TARGET img_transform::img_transform_srv__rosidl_typesupport_c APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(img_transform::img_transform_srv__rosidl_typesupport_c PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "rosidl_runtime_c::rosidl_runtime_c;rosidl_typesupport_c::rosidl_typesupport_c"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libimg_transform__rosidl_typesupport_c.so"
  IMPORTED_SONAME_NOCONFIG "libimg_transform__rosidl_typesupport_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS img_transform::img_transform_srv__rosidl_typesupport_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_img_transform::img_transform_srv__rosidl_typesupport_c "${_IMPORT_PREFIX}/lib/libimg_transform__rosidl_typesupport_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
