project(routing_path)

set(SRC
    routing_path_main.cpp
    ../routing_integration_tests/routing_test_tools.cpp
)

omim_add_executable(${PROJECT_NAME} ${SRC})
omim_link_libraries(
        ${PROJECT_NAME}
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
