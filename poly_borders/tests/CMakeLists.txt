project(poly_borders_postprocessor_tests)

set(
        SRC
        area_tests.cpp
        mark_points_tests.cpp
        remove_empty_spaces_tests.cpp
        tools.cpp
        tools.hpp
)

omim_add_test(${PROJECT_NAME} ${SRC})

target_link_libraries(
        ${PROJECT_NAME}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
)

omim_link_libraries(
        ${PROJECT_NAME}
        poly_borders_postprocessor_library
        platform_tests_support
        generator
        storage
        indexer
        platform
        geometry
        coding
        base
        stats_client
        gflags
        ${CMAKE_DL_LIBS}
        ${LIBZ}
)

link_qt5_core(${PROJECT_NAME})

