macro(common_settings)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

  if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    message(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
  endif()

  add_definitions(-D_SCL_SECURE_NO_WARNINGS)  # VS annoying warnings
  add_definitions(-D_USE_MATH_DEFINES)  # to enable stuff like M_PI

  find_package(OpenCV REQUIRED)
  include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
endmacro()


macro(collect_sources_default)
    file(GLOB SOURCES LIST_DIRECTORIES FALSE src/*.cpp)
  file(GLOB HEADERS LIST_DIRECTORIES FALSE include/${name}/*.hpp)
endmacro()

macro(set_default_properties target folder_name)
  set_target_properties(${target} PROPERTIES FOLDER ${folder_name})
endmacro()

macro(add_library_default name)
  collect_sources_default()
  add_library(${name} ${SOURCES} ${HEADERS})
  include_directories(include)
  set_default_properties(${name} "libs")
  target_include_directories(${name} PUBLIC include)
endmacro()

macro(add_app_default name)
  collect_sources_default()
  add_executable(${name} ${SOURCES} ${HEADERS})
  set_default_properties(${name} "apps")
endmacro()

macro(add_test_default name)
  collect_sources_default()
  add_executable(${name} ${SOURCES} ${HEADERS})
  set_default_properties(${name} "tests")
endmacro()
