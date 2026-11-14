cmake_minimum_required(VERSION 3.21)
project(PacaiCloudCompareCoreAudit LANGUAGES CXX)

option(PACAI_ENABLE_TBB "Build CCCoreLib with oneTBB parallel support" ON)
set(PACAI_CCCORELIB_SOURCE_DIR "" CACHE PATH "Path to the initialized CCCoreLib submodule")

if(NOT PACAI_CCCORELIB_SOURCE_DIR)
    message(FATAL_ERROR
        "Set PACAI_CCCORELIB_SOURCE_DIR to libs/qCC_db/extern/CCCoreLib")
endif()

if(NOT EXISTS "${PACAI_CCCORELIB_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "CCCoreLib CMakeLists.txt not found at: ${PACAI_CCCORELIB_SOURCE_DIR}. "
        "Initialize the CCCoreLib submodule first.")
endif()

# Enforce the product policy before adding the upstream project. If a future
# CCCoreLib change accidentally requests Qt6, CMake must fail rather than find it.
set(CCCORELIB_USE_QT_CONCURRENT OFF CACHE BOOL
    "Pacai forbids QtConcurrent" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_Qt6 TRUE CACHE BOOL
    "Pacai forbids Qt6 discovery in the core build" FORCE)
set(CCCORELIB_USE_CGAL OFF CACHE BOOL
    "CGAL is not part of the initial core isolation POC" FORCE)

if(PACAI_ENABLE_TBB)
    # Require TBB here: upstream CCCoreLib treats TBB as optional and otherwise
    # silently falls back to serial compilation while leaving the option enabled.
    find_package(TBB 2022.0.0 COMPONENTS tbb CONFIG REQUIRED)
    set(CCCORELIB_USE_TBB ON CACHE BOOL
        "Use oneTBB instead of QtConcurrent" FORCE)
else()
    set(CCCORELIB_USE_TBB OFF CACHE BOOL
        "Serial no-Qt isolation build" FORCE)
endif()

add_subdirectory("${PACAI_CCCORELIB_SOURCE_DIR}"
                 "${CMAKE_BINARY_DIR}/cccorelib")

if(CCCORELIB_USE_QT_CONCURRENT)
    message(FATAL_ERROR "Pacai policy violation: CCCORELIB_USE_QT_CONCURRENT is ON")
endif()

if(NOT TARGET CCCoreLib)
    message(FATAL_ERROR "Expected upstream target CCCoreLib was not created")
endif()

# Guard both direct and propagated link dependencies.
foreach(_property LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
    get_target_property(_links CCCoreLib ${_property})
    if(_links AND "${_links}" MATCHES "Qt[56]::")
        message(FATAL_ERROR
            "Pacai policy violation: CCCoreLib ${_property} contains Qt: ${_links}")
    endif()
endforeach()

message(STATUS "Pacai no-Qt CCCoreLib audit target configured")
message(STATUS "CCCORELIB_USE_QT_CONCURRENT=${CCCORELIB_USE_QT_CONCURRENT}")
message(STATUS "CCCORELIB_USE_TBB=${CCCORELIB_USE_TBB}")
