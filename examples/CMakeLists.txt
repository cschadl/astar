add_executable(solve_n_sq_puzzle 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/solve_n_sq_puzzle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cycle_decomposition.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/n_sq_puzzle.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/solve_helpers.hpp)
target_include_directories(solve_n_sq_puzzle PRIVATE ${ASTAR_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
