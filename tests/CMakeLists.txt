add_executable(tests
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/astar_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ida_star_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/shortest_path_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/adj_list_graph.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/grid.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/get_path_cost.h
    ${CMAKE_SOURCE_DIR}/examples/include/cycle_decomposition.hpp
    ${CMAKE_SOURCE_DIR}/examples/include/n_sq_puzzle.hpp
    ${CMAKE_SOURCE_DIR}/examples/include/solve_helpers.hpp)

target_include_directories(tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ASTAR_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/examples/include
    ${GTest_include_dir})

target_link_libraries(tests gtest Threads::Threads)

gtest_discover_tests(tests)