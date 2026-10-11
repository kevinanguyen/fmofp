# Hand-written unit tests. The generated per-message round-trip test lives in
# ../icd/CMakeLists.txt because it is produced there.
add_executable(ofp_tests
    status_test.cpp
    word_test.cpp
    command_word_test.cpp
    status_word_test.cpp
    transport_test.cpp
    osal_test.cpp
    remote_terminal_test.cpp
    schedule_test.cpp
    bus_controller_test.cpp
    flight_control_test.cpp
    gps_solver_test.cpp
    navigation_test.cpp
    icd_messages_test.cpp)
add_dependencies(ofp_tests ofp_icd_generate)
target_link_libraries(ofp_tests PRIVATE
    ofp::bus ofp::osal ofp::icd ofp::bc ofp::rt_process ofp::fcs ofp::nav ofp::warnings
    Threads::Threads GTest::gtest GTest::gtest_main)
add_test(NAME ofp_tests COMMAND ofp_tests)
