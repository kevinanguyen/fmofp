# Operating-system abstraction: the only code in the OFP that talks to the OS.
# One header per facility, one implementation per platform.
add_library(ofp_osal STATIC
    src/time.cpp
    $<IF:$<PLATFORM_ID:Windows>,src/clock_win32.cpp,src/clock_posix.cpp>
    $<IF:$<PLATFORM_ID:Windows>,src/udp_win32.cpp,src/udp_posix.cpp>)
add_library(ofp::osal ALIAS ofp_osal)

target_include_directories(ofp_osal PUBLIC include)
target_link_libraries(ofp_osal PRIVATE ofp::flight PUBLIC ofp::bus)

if(WIN32)
    # High-resolution waitable timers need the Windows 10 API level.
    target_compile_definitions(ofp_osal PRIVATE _WIN32_WINNT=0x0A00 WIN32_LEAN_AND_MEAN NOMINMAX)
    target_link_libraries(ofp_osal PUBLIC ws2_32)
endif()
