# The bus controller: schedule, frame loop, transaction engine, and the
# fmofp_bc process that runs them.
add_library(ofp_bc STATIC
    src/schedule.cpp
    src/icd_schedule.cpp
    src/bus_controller.cpp)
add_library(ofp::bc ALIAS ofp_bc)
add_dependencies(ofp_bc ofp_icd_generate)
target_include_directories(ofp_bc PUBLIC include)
target_link_libraries(ofp_bc PUBLIC ofp::bus ofp::osal ofp::icd PRIVATE ofp::flight)

add_executable(fmofp_bc src/main.cpp)
target_link_libraries(fmofp_bc PRIVATE ofp::bc ofp::warnings)
