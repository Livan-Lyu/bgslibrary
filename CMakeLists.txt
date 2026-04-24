cmake_minimum_required(VERSION 3.10)

project(bgslibrary VERSION 3.3.0)

include(CheckCXXCompilerFlag)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

function(append_if condition value)
  if (${condition})
    foreach(variable ${ARGN})
      set(${variable} "${${variable}} ${value}" PARENT_SCOPE)
    endforeach(variable)
  endif()
endfunction()

#if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR
#    CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR
#    CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
#  check_cxx_compiler_flag("-fvisibility-inlines-hidden" SUPPORTS_FVISIBILITY_INLINES_HIDDEN_FLAG)
#  append_if(SUPPORTS_FVISIBILITY_INLINES_HIDDEN_FLAG "-fvisibility=hidden -fvisibility-inlines-hidden" CMAKE_CXX_FLAGS)
#endif()

if(UNIX)
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
  set(CMAKE_MACOSX_RPATH 1)
endif(UNIX)

# Avoid cmake warnings about changes in behavior of some Mac OS X path
# variable we don't care about.
if (POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

# compilation mode setup
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
  set(BUILD_SHARED_LIBS TRUE)
  #if(BGS_PYTHON_SUPPORT)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
  #else()
  #  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  #  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
  #endif()
endif(WIN32)

set(bgs_out_dir ".")
# First for the generic no-config case (e.g. with mingw)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${bgs_out_dir})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${bgs_out_dir})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${bgs_out_dir})
# Second, for multi-config builds (e.g. msvc)
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
  string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${bgs_out_dir})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${bgs_out_dir})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${bgs_out_dir})
endforeach(OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES)

if(UNIX)
  # add some standard warnings
  ADD_DEFINITIONS(-Wno-variadic-macros -Wno-long-long -Wall -Wextra -Winit-self -Woverloaded-virtual -Wsign-promo -Wno-unused-parameter -pedantic -Woverloaded-virtual -Wno-unknown-pragmas)

  # -ansi does not compile with sjn module
  #ADD_DEFINITIONS(-ansi)

  # if you like to have warinings about conversions, e.g. double->int or double->float etc., or float compare
  #ADD_DEFINITIONS(-Wconversion -Wfloat-equal)

  # Architecture-specific optimization flags.
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(riscv64|riscv32)")
    add_compile_options(-march=rv64gc)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64|i[3-6]86)$")
    add_compile_options(-march=native -mtune=native)
  endif()
endif(UNIX)

# cmake -D BGS_CORE_STATIC=ON ..
if(NOT DEFINED BGS_CORE_STATIC)
  set(BGS_CORE_STATIC OFF)
endif()
message(STATUS "")
message(STATUS "BGS_CORE_STATIC:    ${BGS_CORE_STATIC}")

if(BGS_CORE_STATIC)
  set(OpenCV_STATIC ON)
else()
  set(OpenCV_STATIC OFF)
endif()

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message(STATUS "")
  message(STATUS "OpenCV library status:")
  message(STATUS "    version: ${OpenCV_VERSION}")
  message(STATUS "    libraries: ${OpenCV_LIBS}")
  message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}\n")
endif()

# if(${OpenCV_VERSION} VERSION_EQUAL 3 OR ${OpenCV_VERSION} VERSION_GREATER 3)
#   message(FATAL_ERROR "OpenCV version is not compatible: ${OpenCV_VERSION}")
# endif()

if(${OpenCV_VERSION} VERSION_LESS 2.3.1)
  message(FATAL_ERROR "OpenCV version is not compatible: ${OpenCV_VERSION}")
endif()

file(GLOB main_src bgslibrary/*.cpp bgslibrary/*.c)
file(GLOB main_inc bgslibrary/*.h bgslibrary/*.hpp)

file(GLOB_RECURSE utils_src bgslibrary/utils/*.cpp bgslibrary/utils/*.c)
file(GLOB_RECURSE utils_inc bgslibrary/utils/*.h bgslibrary/utils/*.hpp)

file(GLOB_RECURSE tools_src bgslibrary/tools/*.cpp bgslibrary/tools/*.c)
file(GLOB_RECURSE tools_inc bgslibrary/tools/*.h bgslibrary/tools/*.hpp)

file(GLOB_RECURSE bgs_src bgslibrary/algorithms/*.cpp bgslibrary/algorithms/*.c)
file(GLOB_RECURSE bgs_inc bgslibrary/algorithms/*.h bgslibrary/algorithms/*.hpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

# GMG is not available in older OpenCV versions
if(${OpenCV_VERSION} VERSION_LESS 2.4.3)
  file(GLOB gmg bgslibrary/algorithms/GMG.cpp)
  list(REMOVE_ITEM bgs_src ${gmg})
endif()

if(BGS_CORE_STATIC)
  message(STATUS "Bulding bgslibrary_core STATIC")
  add_library(bgslibrary_core STATIC ${bgs_src} ${tools_src} ${utils_src} ${bgs_inc} ${tools_inc} ${utils_inc})
  #set_property(TARGET bgslibrary_core PROPERTY POSITION_INDEPENDENT_CODE ON)
else()
  message(STATUS "Bulding bgslibrary_core SHARED")
  add_library(bgslibrary_core SHARED ${bgs_src} ${tools_src} ${utils_src} ${bgs_inc} ${tools_inc} ${utils_inc})
  target_link_libraries(bgslibrary_core ${OpenCV_LIBS})
  # generates the export header bgslibrary_core_EXPORTS.h automatically
  include(GenerateExportHeader)
  GENERATE_EXPORT_HEADER(bgslibrary_core
      BASE_NAME bgslibrary_core
      EXPORT_MACRO_NAME bgslibrary_core_EXPORTS
      EXPORT_FILE_NAME bgslibrary_core_EXPORTS.h
      STATIC_DEFINE BGSLIBRARY_CORE_EXPORTS_BUILT_AS_STATIC)
  #set_property(TARGET bgslibrary_core PROPERTY PUBLIC_HEADER ${bgs_inc} ${tools_inc} ${utils_inc})
endif()

#if(WIN32)
#  # set_property(TARGET bgslibrary_core PROPERTY SUFFIX ".lib")
#  #if(BGS_PYTHON_SUPPORT)
#  #  set_property(TARGET bgslibrary_core PROPERTY SUFFIX ".pyd")
#  #endif()
#else()
#  set_property(TARGET bgslibrary_core PROPERTY OUTPUT_NAME "bgs")
add_executable(bgslibrary ${main_src} ${main_inc})
target_link_libraries(bgslibrary ${OpenCV_LIBS} bgslibrary_core)
# set_target_properties(bgslibrary PROPERTIES OUTPUT_NAME bgs)

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/BGSLibrary)
target_include_directories(bgslibrary_core
  PUBLIC
      $<INSTALL_INTERFACE:include>)


install(TARGETS bgslibrary_core
  bgslibrary
  EXPORT bgslibrary_core-targets
  RUNTIME DESTINATION bin COMPONENT app
  LIBRARY DESTINATION lib COMPONENT runtime
  ARCHIVE DESTINATION lib COMPONENT runtime
  #PUBLIC_HEADER DESTINATION include/bgslibrary COMPONENT dev
  FRAMEWORK DESTINATION "/Library/Frameworks"
)
set_target_properties(bgslibrary_core PROPERTIES EXPORT_NAME BGSLibrary)

install(DIRECTORY bgslibrary/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/bgslibrary)
install(EXPORT bgslibrary_core-targets
  FILE
    BGSLibraryTargets.cmake
  NAMESPACE
    BGSLibrary::
  DESTINATION
    ${INSTALL_CONFIGDIR}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/BGSLibraryConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake-modules/BGSLibraryConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/BGSLibraryConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

install(FILES
  ${CMAKE_CURRENT_LIST_DIR}/cmake-modules/FindOpenCV.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/BGSLibraryConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/BGSLibraryConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

if(UNIX)
  # to avoid: error while loading shared libraries: libbgslibrary_core.so
  message(STATUS "You might need to run:")
  message(STATUS "$ LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib")
  message(STATUS "$ export LD_LIBRARY_PATH")
  message(STATUS "after 'make install' to avoid error while loading libbgslibrary_core\n")
endif()

if(WIN32)
  message(STATUS "You might need to add ${CMAKE_CURRENT_BINARY_DIR} to your PATH to be able to run your applications.")
  message(STATUS "> set PATH=%PATH%;${CMAKE_CURRENT_BINARY_DIR}\n")
endif()
