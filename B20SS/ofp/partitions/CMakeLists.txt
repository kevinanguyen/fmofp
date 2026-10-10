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
