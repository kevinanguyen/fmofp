# Compiler rules for the OFP, as two interface targets:
#
#   ofp::warnings   every target, flight code and tests alike
#   ofp::flight     flight code only: adds -fno-exceptions -fno-rtti
#
# Link a target against one of these instead of repeating flags. The flags are
# the rules in CLAUDE.md made mechanical; do not weaken them per target.

add_library(ofp_warnings INTERFACE)
add_library(ofp::warnings ALIAS ofp_warnings)

target_compile_options(ofp_warnings INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wold-style-cast
    -Wcast-align
    -Wcast-qual
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Wimplicit-fallthrough
    -Wundef
    -Wnon-virtual-dtor
    -Woverloaded-virtual
    -Werror)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(ofp_warnings INTERFACE -Wduplicated-cond -Wlogical-op)
endif()

if(OFP_COVERAGE)
    target_compile_options(ofp_warnings INTERFACE --coverage -O0)
    target_link_options(ofp_warnings INTERFACE --coverage)
endif()

if(OFP_SANITIZE)
    target_compile_options(ofp_warnings INTERFACE
        -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
    target_link_options(ofp_warnings INTERFACE -fsanitize=address,undefined)
endif()

add_library(ofp_flight INTERFACE)
add_library(ofp::flight ALIAS ofp_flight)
target_link_libraries(ofp_flight INTERFACE ofp::warnings)
target_compile_options(ofp_flight INTERFACE -fno-exceptions -fno-rtti)
