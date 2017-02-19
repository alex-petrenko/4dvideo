include(ExternalProject)

set(GTEST_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/gtest")
ExternalProject_Add(
    gtest_external
    URL https://github.com/google/googletest/archive/release-1.8.0.zip
    PREFIX ${GTEST_PREFIX}
    CMAKE_ARGS -Dgtest_force_shared_crt=ON
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    LOG_CONFIGURE ON
    LOG_BUILD ON
)

ExternalProject_Get_Property(gtest_external BINARY_DIR)
set(GTEST_BINARY ${BINARY_DIR})
set(GTEST_INCLUDES "${GTEST_PREFIX}/src/gtest_external/googletest/include")

set(GTEST_LIB ${CMAKE_STATIC_LIBRARY_PREFIX}gtest${CMAKE_STATIC_LIBRARY_SUFFIX})
set(GTEST_MAIN_LIB ${CMAKE_STATIC_LIBRARY_PREFIX}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX})

if (MSVC)
    set(GTEST_IMPORTED_LOCATION
        IMPORTED_LOCATION_DEBUG "${GTEST_BINARY}/googlemock/gtest/Debug/${GTEST_LIB}"
        IMPORTED_LOCATION_RELEASE "${GTEST_BINARY}/googlemock/gtest/Release/${GTEST_LIB}"
        IMPORTED_LOCATION_RELWITHDEBINFO "${GTEST_BINARY}/googlemock/gtest/RelWithDebInfo/${GTEST_LIB}")
    set(GTESTMAIN_IMPORTED_LOCATION
        IMPORTED_LOCATION_DEBUG "${GTEST_BINARY}/googlemock/gtest/Debug/${GTEST_MAIN_LIB}"
        IMPORTED_LOCATION_RELEASE "${GTEST_BINARY}/googlemock/gtest/Release/${GTEST_MAIN_LIB}"
        IMPORTED_LOCATION_RELWITHDEBINFO "${GTEST_BINARY}/googlemock/gtest/RelWithDebInfo/${GTEST_MAIN_LIB}")
else()
    set(GTEST_IMPORTED_LOCATION "${GTEST_BINARY}/googlemock/gtest/${GTEST_LIB}")  # not tested on Linux!
    set(GTESTMAIN_IMPORTED_LOCATION "${GTEST_BINARY}/googlemock/gtest/${GTEST_MAIN_LIB}")
endif()

file(MAKE_DIRECTORY ${GTEST_INCLUDES})

# Create a libgtest target to be used as a dependency by test programs.
# GLOBAL is required to extend visibility, by default the target will be visible only inside this directory.
add_library(gtest IMPORTED STATIC GLOBAL)
set_target_properties(
    gtest PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDES}"  # automatically adds GTEST_INCLUDES to every target that links with gtest
    ${GTEST_IMPORTED_LOCATION}
)

add_library(gtest_main IMPORTED STATIC GLOBAL)
set_target_properties(
    gtest_main PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDES}"
    ${GTESTMAIN_IMPORTED_LOCATION}
)

add_dependencies(gtest gtest_external)
