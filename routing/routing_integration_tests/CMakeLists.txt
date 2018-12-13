# This subproject implements integration tests.
# This tests are launched on the whole world dataset.

# It is recommended to place tests here in the following cases:
# - tests are written to be launch on the whole world dataset;
# - tests covers significant number of subsystems;

project(routing_integration_tests)

set(
  SRC
  route_test.cpp
  routing_test_tools.cpp
  routing_test_tools.hpp
)

omim_add_test(${PROJECT_NAME} ${SRC})

omim_link_libraries(
  ${PROJECT_NAME}
  editor_tests_support
  map
  routing
  search
  storage
  mwm_diff
  traffic
  routing_common
  transit
  kml
  editor
  indexer
  platform
  oauthcpp
  geometry
  coding
  base
  jansson
  protobuf
  bsdiff
  succinct
  stats_client
  pugixml
  opening_hours
  icu
  agg
  ${LIBZ}
)

link_qt5_core(${PROJECT_NAME})
link_qt5_network(${PROJECT_NAME})
