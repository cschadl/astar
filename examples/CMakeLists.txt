add_executable(solve_magic_square ${CMAKE_CURRENT_SOURCE_DIR}/src/solve_magic_square.cpp)
target_include_directories(solve_magic_square PRIVATE ${ASTAR_INCLUDE_DIR})

add_executable(solve_n_sq_puzzle 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/solve_n_sq_puzzle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cycle_decomposition.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/n_sq_puzzle.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/solve_helpers.hpp)
target_include_directories(solve_n_sq_puzzle PRIVATE ${ASTAR_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(test_a_star_search ${CMAKE_CURRENT_SOURCE_DIR}/src/test_a_star_search.cpp)
target_include_directories(test_a_star_search PRIVATE ${ASTAR_INCLUDE_DIR})

add_executable(test_cycle_decomposition ${CMAKE_CURRENT_SOURCE_DIR}/src/test_cycle_decomposition.cpp)
target_include_directories(test_cycle_decomposition PRIVATE ${ASTAR_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
