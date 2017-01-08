project(libtri VERSION 0.1 LANGUAGES CXX)

add_library_default(tri)
target_link_libraries(tri util ${OpenCV_LIBS})