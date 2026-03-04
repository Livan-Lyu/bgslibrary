#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "BGSLibrary::BGSLibrary" for configuration "Release"
set_property(TARGET BGSLibrary::BGSLibrary APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(BGSLibrary::BGSLibrary PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libbgslibrary_core.so"
  IMPORTED_SONAME_RELEASE "libbgslibrary_core.so"
  )

list(APPEND _cmake_import_check_targets BGSLibrary::BGSLibrary )
list(APPEND _cmake_import_check_files_for_BGSLibrary::BGSLibrary "${_IMPORT_PREFIX}/lib/libbgslibrary_core.so" )

# Import target "BGSLibrary::bgslibrary" for configuration "Release"
set_property(TARGET BGSLibrary::bgslibrary APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(BGSLibrary::bgslibrary PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/bgslibrary"
  )

list(APPEND _cmake_import_check_targets BGSLibrary::bgslibrary )
list(APPEND _cmake_import_check_files_for_BGSLibrary::bgslibrary "${_IMPORT_PREFIX}/bin/bgslibrary" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
