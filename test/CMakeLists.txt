include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/965f8ecbfd8b91bbd4f5ee4914c028660bb89029.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt
    ON
    CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(HvilInterlockTest testhvil.cpp)
target_link_libraries(HvilInterlockTest PRIVATE global_options libopeninv link_last)
add_test(NAME HvilInterlockTest COMMAND HvilInterlockTest)

add_executable(OverCurrentTripTest testovercurrent.cpp)
target_link_libraries(OverCurrentTripTest PRIVATE global_options libopeninv link_last)
add_test(NAME OverCurrentTripTest COMMAND OverCurrentTripTest)

add_executable(DcLinkTripTest testdclinktrip.cpp)
target_link_libraries(DcLinkTripTest PRIVATE global_options libopeninv link_last)
add_test(NAME DcLinkTripTest COMMAND DcLinkTripTest)

# Files on the control ISR path; on the C28x a plain int is only 16 bits.
set(ISR_PATH_FILES
    include/focpwmgeneration.h
    include/pwmgenerationbase.h
    include/sinepwmgeneration.h
    include/resolverencoder.h
    include/resolversignalmonitor.h
    include/overcurrenttrip.h
    include/dclinktrip.h
    include/hvilinterlock.h
    libopeninv/include/foc.h
    libopeninv/src/foc.cpp
    libopeninv/include/picontroller.h
    libopeninv/src/picontroller.cpp
    libopeninv/include/sine_core.h
    libopeninv/src/sine_core.cpp
    platform/c2000/inverter_drivers/pwmdriver.cpp
    platform/c2000/inverter_drivers/encoder.cpp
    platform/c2000/inverter_drivers/c2000/encoder.h
    platform/c2000/inverter_drivers/motoranalogcapture.cpp)
list(TRANSFORM ISR_PATH_FILES PREPEND ${PROJECT_SOURCE_DIR}/)
add_executable(IsrIntWidthTest testisrintwidth.cpp)
target_link_libraries(IsrIntWidthTest PRIVATE global_options)
add_test(NAME IsrIntWidthTest COMMAND IsrIntWidthTest ${ISR_PATH_FILES})

# Unit test the library
file(
  GLOB
  SRCS
  test_fp.cpp
  test_fu.cpp
  test_throttle.cpp
  testcrc8.cpp
  testfoccalc.cpp
  testfocpwmgeneration.cpp
  testsinecore.cpp
  testresolverencoder.cpp
  testresolversignal.cpp
  mockencoder.cpp
  mockpwmdriver.cpp
  mockresolversample.cpp)
add_executable(OpenInverterTest ${SRCS})

target_link_libraries(OpenInverterTest PUBLIC global_options libopeninv
                                              portablesine)

# Create a code coverage utility target - assumes that gcov and lcov are
# available TODO: Test for these utilities
include(CodeCoverage)
append_coverage_compiler_flags()
setup_target_for_coverage_lcov(
  NAME
  coverage_OpenInverterTest
  EXECUTABLE
  OpenInverterTest
  OUTPUT
  coverage_OpenInverterTest_dir
  EXCLUDE
  "/usr/include/*"
  "build/_deps/*")

# Generic Google Test libraries
target_link_libraries(OpenInverterTest PUBLIC gtest_main gmock_main)

include(GoogleTest)
gtest_discover_tests(OpenInverterTest)
