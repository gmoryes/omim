project(poly_borders_postprocessor)

include_directories(${OMIM_ROOT}/3party/gflags/src)

set(SRC main.cpp)

omim_add_executable(${PROJECT_NAME} ${SRC})

omim_link_libraries(
        ${PROJECT_NAME}
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
link_qt5_network(${PROJECT_NAME})
