project(poly_borders_tests)

set(
  SRC
  mark_points_tests.cpp
  remove_empty_spaces_tests.cpp
  tools.cpp
  tools.hpp
)

omim_add_test(${PROJECT_NAME} ${SRC})

omim_link_libraries(
  ${PROJECT_NAME}
  poly_borders
  platform_tests_support
  generator
  routing
  traffic
  routing_common
  storage
  indexer
  platform
  geometry
  coding
  base
  icu
  jansson
  protobuf
  stats_client
  tess2
  gflags
  oauthcpp
  sqlite3
  ${CMAKE_DL_LIBS}
  ${LIBZ}
)

link_qt5_core(${PROJECT_NAME})

