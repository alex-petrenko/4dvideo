project(libtri VERSION 0.1 LANGUAGES CXX)

set(SOURCES
    src/triangulation.cpp
)

set(HEADERS
    # include/tri/
)

add_library(tri ${SOURCES} ${HEADERS})
set_target_properties(tri PROPERTIES LINKER_LANGUAGE CXX)

include_directories(include)