# Code shared by every partition process, and the partitions themselves.
# Today: the terminal socket loop and a stub terminal that answers TERM_STATUS,
# which is what the Phase 1 gate runs against the bus controller.
add_library(ofp_rt_process STATIC common/src/rt_process.cpp)
add_library(ofp::rt_process ALIAS ofp_rt_process)
target_include_directories(ofp_rt_process PUBLIC common/include)
target_link_libraries(ofp_rt_process PUBLIC ofp::bus ofp::osal PRIVATE ofp::flight)

add_executable(fmofp_rt_stub rt_stub/src/main.cpp)
add_dependencies(fmofp_rt_stub ofp_icd_generate)
target_link_libraries(fmofp_rt_stub PRIVATE ofp::rt_process ofp::bc ofp::icd ofp::warnings)

# Flight control: the first ported partition. The logic is a library so the
# tests can drive it frame by frame; fmofp_fcs puts it on the bus.
add_library(ofp_fcs STATIC fcs/src/flight_control.cpp)
add_library(ofp::fcs ALIAS ofp_fcs)
add_dependencies(ofp_fcs ofp_icd_generate)
target_include_directories(ofp_fcs PUBLIC fcs/include)
target_link_libraries(ofp_fcs PUBLIC ofp::bus ofp::icd PRIVATE ofp::flight)

add_executable(fmofp_fcs fcs/src/main.cpp)
target_link_libraries(fmofp_fcs PRIVATE ofp::fcs ofp::rt_process ofp::bc ofp::warnings)

# Navigation: the GPS solver and the GPS / dead-reckoning fusion as a library,
# fmofp_nav on the bus, and fmofp_nav_tool, which exposes the pure functions on
# stdin/stdout so the Python tests can run the original Python beside them.
add_library(ofp_nav STATIC nav/src/gps_solver.cpp nav/src/navigation.cpp)
add_library(ofp::nav ALIAS ofp_nav)
add_dependencies(ofp_nav ofp_icd_generate)
target_include_directories(ofp_nav PUBLIC nav/include)
target_link_libraries(ofp_nav PUBLIC ofp::bus ofp::icd PRIVATE ofp::flight)

add_executable(fmofp_nav nav/src/main.cpp)
target_link_libraries(fmofp_nav PRIVATE ofp::nav ofp::rt_process ofp::bc ofp::warnings)

add_executable(fmofp_nav_tool nav/tools/nav_tool.cpp)
target_link_libraries(fmofp_nav_tool PRIVATE ofp::nav ofp::warnings)
