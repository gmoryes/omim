project(poly_borders_postprocessor_library)

set(
        SRC
        borders_data.cpp
        borders_data.hpp
        help_structures.cpp
        help_structures.hpp
)

omim_add_library(${PROJECT_NAME} ${SRC})
