#include <gtest/gtest.h>

#include "adj_list_graph.h"
#include "grid.h"

#include <astar/a_star_search.hpp>
#include <n_sq_puzzle.hpp>
#include <solve_helpers.hpp>

#include <algorithm>
#include <functional>
#include <utility>
#include <unordered_set>
#include <limits>

using namespace cds;
namespace ph = std::placeholders;

namespace
{
	int null_heuristic(char)
	{
		return 0;
	}

	bool is_goal(char n)
	{
		return n == 'z';
	}

	template <typename Iterator, typename NeighborWeightFn,
		typename ValueType = decltype(std::declval<NeighborWeightFn>()(*std::declval<Iterator>(), *std::declval<Iterator>()))>
	ValueType get_path_cost(Iterator begin, Iterator end, NeighborWeightFn weight_fn)
	{
		return std::inner_product(begin, std::prev(end),
			std::next(begin),
			ValueType(0), std::plus<>(), weight_fn);
	}
}

TEST(AStarTest, DijkstraShortestPath)
{
	adj_list_graph_t graph = {
		{ 'a', {{'b', 4 }, {'c', 3}} },
		{ 'b', {{'a', 4 }, {'e', 12}, {'f', 5 }} },
		{ 'c', {{'a', 3 }, {'d', 7 }, {'e', 10}} },
		{ 'd', {{'c', 7 }, {'e', 2 }} },
		{ 'e', {{'c', 10}, {'d', 2 }, {'b', 12}, {'z', 5}} },
		{ 'f', {{'b', 5 }, {'z', 16}} },
		{ 'z', {{'f', 16}, {'e', 5 }} }
	};

	auto expand_fn = [&graph](char node)
	{
		return expand_adj_list_graph(graph, node);
	};

	auto neighbor_weight_fn = [&graph](char n, char m)
	{
		return neighbor_weight(graph, n, m);
	};

	char start_node = 'a';

	std::vector<char> path;
	int path_cost;

	bool found_path = astar::a_star_search(
		start_node, expand_fn, &null_heuristic, neighbor_weight_fn, &is_goal,
		std::back_inserter(path), &path_cost);
	
	ASSERT_TRUE(found_path);
	ASSERT_EQ(path.size(), 5);
	
	auto p_it = path.begin();
	EXPECT_EQ(*(p_it++), 'a');
	EXPECT_EQ(*(p_it++), 'c');
	EXPECT_EQ(*(p_it++), 'd');
	EXPECT_EQ(*(p_it++), 'e');
	EXPECT_EQ(*(p_it++), 'z');

	auto computed_path_cost = get_path_cost(path.begin(), path.end(), neighbor_weight_fn);

	EXPECT_EQ(computed_path_cost, 17);
	EXPECT_EQ(path_cost, computed_path_cost);
}

namespace
{
	std::vector<grid_node> expand_grid(
		std::unordered_set<grid_node> const& obstacle_nodes,
		grid_node const& grid_min, grid_node const& grid_max,
		grid_node const& n)
	{
		std::vector<grid_node> expand_nodes;

		std::vector<int> node_deltas = { -1, 0, 1 };
		for (int i = 0 ; i < node_deltas.size() ; i++)
		{
			for (int j = 0 ; j < node_deltas.size(); j++)
			{
				if (i == 0 && j == 0)
					continue;

				grid_node expand_node{ n.x + node_deltas[i], n.y + node_deltas[j] };

				if (obstacle_nodes.find(expand_node) != obstacle_nodes.end())
					continue;

				if (expand_node.x < grid_min.x || expand_node.y < grid_min.y)
					continue;

				if (expand_node.y > grid_max.x || expand_node.y > grid_max.y)
					continue;

				expand_nodes.emplace_back(std::move(expand_node));
			}
		}

		return expand_nodes;
	}

	double node_dist(grid_node const& n1, grid_node const& n2)
	{
		double const dx = (double) n1.x - (double) n2.x;
		double const dy = (double) n1.y - (double) n2.y;

		return sqrt((dx*dx) + (dy*dy));
	}
}

TEST(AStarTest, ShortestPathGrid)
{
	// grid starts at (0,0) in upper left corner, ends at (7, 7) in lower right
	std::unordered_set<grid_node> obstacles = {
		grid_node{0, 2}, grid_node{ 0, 3 },
		grid_node{1, 2},
		grid_node{3, 2},
		grid_node{4, 0}, grid_node{4, 2},
		grid_node{5, 0}, grid_node{5, 1},
		grid_node{6, 0}, grid_node{6, 1},
		grid_node{7, 0}, grid_node{7, 1}
	};

	grid_node const start_node{0, 0};
	grid_node const goal_node{7, 3};

	auto expand_fn = [&obstacles](grid_node const& n)
	{
		return expand_grid(obstacles, grid_node{0, 0}, grid_node{7, 7}, n);
	};

	auto h_fn = [&goal_node](grid_node const& n)
	{
		return node_dist(n, goal_node);
	};

	auto goal_fn = [&goal_node](grid_node const&n )
	{
		return n == goal_node;
	};

	std::vector<grid_node> path;
	double path_cost;

	bool found_path = astar::a_star_search(
		grid_node{0, 0}, expand_fn, h_fn, node_dist, goal_fn,
		std::back_inserter(path), &path_cost);

	ASSERT_TRUE(found_path);
	ASSERT_FALSE(path.empty());

	double computed_path_cost = get_path_cost(path.begin(), path.end(), node_dist);

	EXPECT_EQ(path.size(), 8);
	EXPECT_NEAR(computed_path_cost, sqrt(2) * 3 + 4, std::numeric_limits<double>::epsilon() * 100);
	EXPECT_EQ(computed_path_cost, path_cost);

	EXPECT_EQ(path.front(), start_node);
	EXPECT_EQ(path.back(), goal_node);
	
	EXPECT_TRUE(std::none_of(path.begin(), path.end(), 
		[&obstacles](grid_node const& n) { return obstacles.find(n) != obstacles.end(); }));

	EXPECT_TRUE(std::all_of(path.begin(), path.end(),
		[](grid_node const& n) { return n.x >= 0 && n.y >= 0 && n.x <= 7 && n.y <= 7; }));
}

TEST(AStarSearch, NSqPuzzle)
{
	n_sq_puzzle<3> puzzle;
	puzzle.set({7, 2, 4, 3, 0, 1, 8, 5, 6});

	n_sq_puzzle<3> goal_state;
	auto is_goal = [&goal_state](n_sq_puzzle<3> const& n) { return n == goal_state; };

	auto heuristic_fn = [&goal_state](n_sq_puzzle<3> const& n) { return tile_taxicab_dist(n, goal_state); };

	auto dist_fn = [](n_sq_puzzle<3> const&, n_sq_puzzle<3> const&) { return 1; };

	std::vector<n_sq_puzzle<3>> path;

	bool found_path = astar::a_star_search(
		puzzle, &expand<3>, heuristic_fn, dist_fn, is_goal,
		std::back_inserter(path));

	ASSERT_TRUE(found_path);

	using state_t = n_sq_puzzle<3>::state_t;

	std::vector<n_sq_puzzle<3>> expected_path;
	add_puzzle_state<3>(expected_path, { 7, 2, 4, 3, 0, 1, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 7, 0, 4, 3, 2, 1, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 7, 4, 0, 3, 2, 1, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 7, 4, 1, 3, 2, 0, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 7, 4, 1, 3, 0, 2, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 7, 4, 1, 0, 3, 2, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 0, 4, 1, 7, 3, 2, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 4, 0, 1, 7, 3, 2, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 4, 1, 0, 7, 3, 2, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 4, 1, 2, 7, 3, 0, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 4, 1, 2, 7, 0, 3, 8, 5, 6 });
	add_puzzle_state<3>(expected_path, { 4, 1, 2, 7, 5, 3, 8, 0, 6 });
	add_puzzle_state<3>(expected_path, { 4, 1, 2, 7, 5, 3, 0, 8, 6 });
	add_puzzle_state<3>(expected_path, { 4, 1, 2, 0, 5, 3, 7, 8, 6 });
	add_puzzle_state<3>(expected_path, { 0, 1, 2, 4, 5, 3, 7, 8, 6 });
	add_puzzle_state<3>(expected_path, { 1, 0, 2, 4, 5, 3, 7, 8, 6 });
	add_puzzle_state<3>(expected_path, { 1, 2, 0, 4, 5, 3, 7, 8, 6 });
	add_puzzle_state<3>(expected_path, { 1, 2, 3, 4, 5, 0, 7, 8, 6 });
	add_puzzle_state<3>(expected_path, { 1, 2, 3, 4, 5, 6, 7, 8, 0 });

	EXPECT_FALSE(path.empty());
	EXPECT_EQ(path, expected_path);
}
