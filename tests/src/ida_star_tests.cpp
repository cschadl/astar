#include <gtest/gtest.h>

#include <astar/ida_star_search.hpp>

#include <n_sq_puzzle.hpp>
#include <solve_helpers.hpp>

#include <algorithm>

using namespace cds;

TEST(IdaStar, FifteenPuzzle)
{
	n_sq_puzzle<4> puzzle;
	puzzle.set({ 12, 5, 7, 8, 1, 3, 11, 15, 9, 13, 6, 14, 2, 0, 4, 10 });

	n_sq_puzzle<4> goal_state;
	auto is_goal = [&goal_state](n_sq_puzzle<4> const& n) { return n == goal_state; };

	auto heuristic_fn = [&goal_state](n_sq_puzzle<4> const& n) { return tile_taxicab_dist(n, goal_state); };

	auto dist_fn = [](n_sq_puzzle<4> const&, n_sq_puzzle<4> const&) { return 1; };

	std::vector<n_sq_puzzle<4>> path;

	bool found_path = astar::ida_star_search(
		puzzle, &expand<4>, heuristic_fn, dist_fn, is_goal,
		std::back_inserter(path));

	ASSERT_TRUE(found_path);

	EXPECT_EQ(path.size(), 45);
	EXPECT_EQ(path.front(), puzzle);
	EXPECT_TRUE(path.back().is_solved());

	using MoveType = n_sq_puzzle<4>::MoveType;
	for (auto p_it = path.begin(); p_it != std::prev(path.end()); ++p_it)
	{
		bool found_move = false;

		for (auto move : { MoveType::UP, MoveType::DOWN, MoveType::LEFT, MoveType::RIGHT })
		{
			EXPECT_FALSE(*p_it == *std::next(p_it)) << *p_it;
			
			if (p_it->moved(move) == *std::next(p_it))
			{
				found_move = true;
				break;
			}
		}

		EXPECT_TRUE(found_move) << *p_it << " cannot be moved to " << *std::next(p_it);
	}
}