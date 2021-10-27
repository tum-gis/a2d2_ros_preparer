cmake_minimum_required(VERSION 3.0.2)

project(a2d2_ros_preparer_docs)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules")
find_package(Sphinx)

if(SPHINX_FOUND)
    set(OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/html")

    add_custom_target(build_doc ALL
            ${SPHINX_EXECUTABLE} -b html
            ${CMAKE_CURRENT_SOURCE_DIR}/source
            ${CMAKE_CURRENT_BINARY_DIR}/html
            COMMENT "Building documentation."
            )
endif()
