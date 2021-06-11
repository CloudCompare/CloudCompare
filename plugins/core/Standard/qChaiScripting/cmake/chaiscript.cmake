set(CHAISCRIPT_VERSION 6.1.0)
find_package(chaiscript ${CHAISCRIPT_VERSION} QUIET)

if (NOT chaiscript_FOUND)
  include(FetchContent)

  FetchContent_Declare(
    chaiscript
    GIT_REPOSITORY https://github.com/ChaiScript/ChaiScript.git
    GIT_TAG v${CHAISCRIPT_VERSION}
  )

  FetchContent_GetProperties(chaiscript)
  if (NOT chaiscript_POPULATED)
    set(FETCHCONTENT_QUIET NO)
    FetchContent_Populate(chaiscript)

    set(MULTITHREAD_SUPPORT_ENABLED OFF CACHE BOOL "" FORCE)


    set(BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_MODULES ON CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(BUILD_LIBFUZZ_TESTER OFF CACHE BOOL "" FORCE)
    
    add_subdirectory(${chaiscript_SOURCE_DIR} ${chaiscript_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
endif()