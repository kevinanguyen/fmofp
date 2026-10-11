# Bus word primitives. Flight code: no exceptions, no RTTI, no heap.
add_library(ofp_bus STATIC
    src/status.cpp
    src/word.cpp
    src/command_word.cpp
    src/status_word.cpp
    src/transport.cpp
    src/remote_terminal.cpp
    src/block.cpp)
add_library(ofp::bus ALIAS ofp_bus)

target_include_directories(ofp_bus PUBLIC include)
target_link_libraries(ofp_bus PRIVATE ofp::flight)
