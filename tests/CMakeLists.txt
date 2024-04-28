add_executable(tests
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/astar_tests.cpp)

target_include_directories(tests PRIVATE
    ${ASTAR_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/examples/include
    ${GTest_include_dir})

target_link_libraries(tests gtest Threads::Threads)

gtest_discover_tests(tests)