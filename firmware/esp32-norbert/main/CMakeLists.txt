file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/main.c" "src/*.c" "src/**/*.c")
idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS "include")
