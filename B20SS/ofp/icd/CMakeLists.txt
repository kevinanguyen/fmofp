# C++ generated from the ICD at build time. The XML is the only source of a
# message layout; nothing produced here is checked in.
set(OFP_ICD_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(OFP_ICD_GENERATED_HEADERS
    "${OFP_ICD_GEN_DIR}/include/fmofp/icd/version.hpp"
    "${OFP_ICD_GEN_DIR}/include/fmofp/icd/enums.hpp"
    "${OFP_ICD_GEN_DIR}/include/fmofp/icd/messages.hpp"
    "${OFP_ICD_GEN_DIR}/include/fmofp/icd/blocks.hpp")
set(OFP_ICD_GENERATED_TEST "${OFP_ICD_GEN_DIR}/icd_roundtrip_test.cpp")

add_custom_command(
    OUTPUT ${OFP_ICD_GENERATED_HEADERS} ${OFP_ICD_GENERATED_TEST}
    COMMAND ${Python3_EXECUTABLE} ${OFP_ICD_TOOL} gen-cpp --icd ${OFP_ICD_XML} --out ${OFP_ICD_GEN_DIR}
    DEPENDS ${OFP_ICD_XML} ${OFP_ICD_TOOL}
    COMMENT "Generating C++ from ${OFP_ICD_XML}"
    VERBATIM)
add_custom_target(ofp_icd_generate DEPENDS ${OFP_ICD_GENERATED_HEADERS} ${OFP_ICD_GENERATED_TEST})

add_library(ofp_icd INTERFACE)
add_library(ofp::icd ALIAS ofp_icd)
add_dependencies(ofp_icd ofp_icd_generate)
target_include_directories(ofp_icd INTERFACE "${OFP_ICD_GEN_DIR}/include")
target_link_libraries(ofp_icd INTERFACE ofp::bus)

if(OFP_BUILD_TESTS)
    add_executable(ofp_icd_roundtrip_test ${OFP_ICD_GENERATED_TEST})
    add_dependencies(ofp_icd_roundtrip_test ofp_icd_generate)
    target_link_libraries(ofp_icd_roundtrip_test PRIVATE
        ofp::icd ofp::warnings GTest::gtest GTest::gtest_main)
    add_test(NAME ofp_icd_roundtrip_test COMMAND ofp_icd_roundtrip_test)
endif()
