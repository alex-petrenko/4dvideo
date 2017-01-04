include_directories(${libtri_SOURCE_DIR}/include)

set(SOURCES
    src/tri_tests.cpp
)

set(HEADERS
)

add_executable(test_app ${SOURCES} ${HEADERS})
target_link_libraries(test_app gtest gtest_main tri)