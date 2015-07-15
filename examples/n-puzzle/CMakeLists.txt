set(TARGET_NAME n-puzzle)
set(SOURCE_DIR src)
set(DEST_DIR bin)
set(INCLUDE_DIRS include)
file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)

include_directories(${INCLUDE_DIRS})
add_executable(${TARGET_NAME} ${SOURCES})

# output binary to DEST_DIR
set_target_properties(${TARGET_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${DEST_DIR}
    )
# make sure DEST_DIR exists before outputting there
add_custom_target(create-runtime-output-directory
    COMMAND ${CMAKE_COMMAND} -E make_directory ${DEST_DIR})
add_dependencies(${TARGET_NAME} create-runtime-output-directory)

set(TARGET_NAME astar-algorithm)
set(SOURCE_DIR src/astar-algorithm-cpp)
set(DEST_DIR bin)
file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)

include_directories(${INCLUDE_DIRS})
add_executable(${TARGET_NAME} ${SOURCES})

# output binary to DEST_DIR
set_target_properties(${TARGET_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${DEST_DIR}
    )
# make sure DEST_DIR exists before outputting there
add_dependencies(${TARGET_NAME} create-runtime-output-directory)
