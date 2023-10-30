file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "main.c" "src/**/*.c")
idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS "include")
