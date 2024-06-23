# ---------------------------------------------
# CMake configuration for slam_vdb_ros
# ---------------------------------------------
# author - Sriharsha Ghanta (ghanta1996@gmail.com)
# --------------------------------------------

# The minimum required cmake version
cmake_minimum_required(VERSION 3.16)

project(slam_vdb VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release) # None, Debug, Release, RelWithDebInfo, MinSizeRel

set(CMAKE_CXX_STANDARD 17) # ensure cmake instructs compiler to use C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON) # ensure the C++ standard given before is actually used
set(CMAKE_CXX_EXTENSIONS OFF) # avoid compile flags of the type -std=gnu++1z add

# Solution Folder options:
if(${CMAKE_SOURCE_DIR} STREQUAL ${PROJECT_SOURCE_DIR})
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif()

# find all dependencies
find_package(Eigen3 REQUIRED)

# -----------------------------------------------------------------------
# Options
# Set -DBUILD_TESTING=ON for compiling unit tests
# Set -DCODE_COVERAGE=ON to generate a code coverage reports when unit tests are executed
# Set -DBUILD_BENCHMARKS=ON for compiling benchmarks files
# Set -DBUILD_DOCUMENTATION=ON to create docs target. Doxygen documentation can be generated with "make docs"
# -----------------------------------------------------------------------
# Build dynamically linkable libraries
unset(BUILD_SHARED_LIBS)
option(BUILD_SHARED_LIBS "Build sharedlibs" ON)
message(STATUS "Build sharedlibs: " ${BUILD_SHARED_LIBS})

# Build unit test files - Deafult is OFF
unset(BUILD_TESTING)
option(BUILD_TESTING "Build testing" OFF)
message(STATUS "Build testing: " ${BUILD_TESTING})

# Build code coverage files - Deafult is OFF
unset(CODE_COVERAGE)
option(CODE_COVERAGE "Enable coverage reporting" OFF)
message(STATUS "Enable coverage reporting: " ${CODE_COVERAGE})

# Build code benchmark files - Deafult is OFF
unset(BUILD_BENCHMARK)
option(BUILD_BENCHMARK "Build benchmark" OFF)
message(STATUS "Build benchmark: " ${BUILD_BENCHMARK})

# Build doxygen target to create docs - Deafult is OFF
unset(BUILD_DOCUMENTATION)
option(BUILD_DOCUMENTATION "Build Documentation" OFF)
message(STATUS "Build Documentation: " ${BUILD_DOCUMENTATION})

# -----------------------------------------------------------------------
# Target
# -----------------------------------------------------------------------
# If individual libraries have different versions  - set them in set_target_properties
# Target linked with interface

# Define the library
add_library(slam_vdb "")
add_library(slam_vdb::slam_vdb ALIAS slam_vdb)

# Set compile options
target_compile_options(slam_vdb PRIVATE
	"$<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-ffp-contract=off>"
	"$<$<CXX_COMPILER_ID:GNU>:-ffp-contract=off>"
	"$<$<CXX_COMPILER_ID:MSVC>:/fp:precise>"
)

target_include_directories(slam_vdb
	PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
	$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(slam_vdb
	INTERFACE Eigen3::Eigen
)

# Add the sources to the target
file(GLOB_RECURSE _sources ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp CONFIGURE_DEPEND)
target_sources(slam_vdb PRIVATE ${_sources})
unset(_sources)

if(BUILD_SHARED_LIBS)
	set_target_properties(slam_vdb PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
endif()

# -----------------------------------------------------------------------
# Build Benchmark
# -----------------------------------------------------------------------
if(BUILD_BENCHMARK)
	add_subdirectory(benchmark)
endif(BUILD_BENCHMARK)

# -----------------------------------------------------------------------
# Build documentation
# -----------------------------------------------------------------------
if(BUILD_DOCUMENTATION)
	add_subdirectory(documentation)
endif(BUILD_DOCUMENTATION)

# -----------------------------------------------------------------------
# Set Code Coverage
# -----------------------------------------------------------------------
if(CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
	message(STATUS "Code Coverage: ${CODE_COVERAGE}")

	# Add required flags (GCC & LLVM/Clang)
	include(cmake/CodeCoverage.cmake)
	append_coverage_compiler_flags()
endif(CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")

# -----------------------------------------------------------------------
# Build Tests
# -----------------------------------------------------------------------
if(BUILD_TESTING)
	add_subdirectory(tests)
endif(BUILD_TESTING)

# -----------------------------------------------------------------------------
# Setup build directories
# -----------------------------------------------------------------------------
# Place executables and shared libraries in the same location for
# convenience of direct execution from common spot and for
# convenience in environments without RPATH support.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
message(STATUS "SLAM_VDB: Run-time output: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "SLAM_VDB: Archives output: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")

# -----------------------------------------------------------------------
# Install
# If the SLAMVDB_INSTALL_DIR is defined CMAKE_INSTALL_PREFIX is set with given value
# -----------------------------------------------------------------------
if(DEFINED SLAMVDB_INSTALL_DIR)
	set(CMAKE_INSTALL_PREFIX ${SLAMVDB_INSTALL_DIR})
endif(DEFINED SLAMVDB_INSTALL_DIR)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

message(STATUS "CMake install dir: " ${CMAKE_INSTALL_PREFIX})

# Set relative install directories
set(INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_INCLUDEDIR}")
set(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake/")

# Create the cmake config file
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY AnyNewerVersion)

configure_file(cmake/${PROJECT_NAME}-config.cmake
	"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
	COPYONLY)

install(TARGETS slam_vdb
	EXPORT ${PROJECT_NAME}-targets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ${PROJECT_NAME}-targets
	FILE ${PROJECT_NAME}-targets.cmake
	NAMESPACE SLAM_VDB::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(FILES
	"${CMAKE_CURRENT_BINARY_DIR}/slam_vdb-config.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/slam_vdb-config-version.cmake"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(DIRECTORY
	"${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}"
	"${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	FILES_MATCHING PATTERN "*.hpp")

# -----------------------------------------------------------------------
# Uninstall
# -----------------------------------------------------------------------
add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
	TARGET "uninstall"
	POST_BUILD
	COMMENT "Uninstalling slam_vdb"
	COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
	install_manifest.txt to be uninstalled!
)