#include <gtest/gtest.h>

#include "adj_list_graph.h"
#include "get_path_cost.h"

#include <astar/a_star_search.hpp>
#include <astar/ida_star_search.hpp>
#include <n_sq_puzzle.hpp>
#include <solve_helpers.hpp>

#include <algorithm>
#include <functional>
#include <utility>

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

	adj_list_graph_t const theGraph = {
		{ 'a', {{'b', 4 }, {'c', 3}} },
		{ 'b', {{'a', 4 }, {'e', 12}, {'f', 5 }} },
		{ 'c', {{'a', 3 }, {'d', 7 }, {'e', 10}} },
		{ 'd', {{'c', 7 }, {'e', 2 }} },
		{ 'e', {{'c', 10}, {'d', 2 }, {'b', 12}, {'z', 5}} },
		{ 'f', {{'b', 5 }, {'z', 16}} },
		{ 'z', {{'f', 16}, {'e', 5 }} }
	};

	class GraphSearchTest
	{
	protected:
		adj_list_graph_t m_graph;

		std::vector<char> expand(char node)
		{
			return expand_adj_list_graph(m_graph, node);
		}

		int neighbor_weight(char n, char m)
		{
			return ::neighbor_weight(m_graph, n, m);
		}

	public:
		GraphSearchTest(adj_list_graph_t const& graph)
			: m_graph(graph)
		{

		}

		~GraphSearchTest() { }

		adj_list_graph_t const& get_graph() const { return m_graph; }

		virtual bool doSearch(char start_node, std::vector<char>& out_path, int& out_path_cost) = 0;
	};

	class AStarGraphSearchTest : public GraphSearchTest
	{
	public:
		AStarGraphSearchTest()
			: GraphSearchTest(theGraph)
		{

		}

		bool doSearch(char start_node, std::vector<char>& out_path, int& out_path_cost)
		{
			return astar::a_star_search(
				start_node,
				[this](char n) { return this->expand(n); },
				&null_heuristic,
				[this](char n, char m) { return this->neighbor_weight(n, m); },
				&is_goal,
				std::back_inserter(out_path),
				&out_path_cost
			);
		}
	};

	class IDAStarGraphSearchTest : public GraphSearchTest
	{
	public:
		IDAStarGraphSearchTest()
			: GraphSearchTest(theGraph)
		{

		}

		bool doSearch(char start_node, std::vector<char>& out_path, int& out_path_cost)
		{
			return astar::ida_star_search(
				start_node,
				[this](char n) { return this->expand(n); },
				&null_heuristic,
				[this](char n, char m) { return this->neighbor_weight(n, m); },
				&is_goal,
				std::back_inserter(out_path),
				&out_path_cost
			);
		}
	};
}

template <typename T>
class DijkstraGraphSearchTest : public testing::Test
{
protected:
	T theTest;
};

using DijkstraGraphSearchImplementations = 
	testing::Types<AStarGraphSearchTest, IDAStarGraphSearchTest>;

TYPED_TEST_SUITE(DijkstraGraphSearchTest, DijkstraGraphSearchImplementations);

TYPED_TEST(DijkstraGraphSearchTest, DijkstraGraphSearchImplementations)
{
	char start_node = 'a';

	std::vector<char> path;
	int path_cost;

	bool found_path = this->theTest.doSearch(start_node, path, path_cost);
	
	ASSERT_TRUE(found_path);
	ASSERT_EQ(path.size(), 5);
	
	auto p_it = path.begin();
	EXPECT_EQ(*(p_it++), 'a');
	EXPECT_EQ(*(p_it++), 'c');
	EXPECT_EQ(*(p_it++), 'd');
	EXPECT_EQ(*(p_it++), 'e');
	EXPECT_EQ(*(p_it++), 'z');

	auto const& graph = this->theTest.get_graph();
	auto computed_path_cost = get_path_cost(path.begin(), path.end(),
		[&graph] (char n, char m) { return ::neighbor_weight(graph, n, m); });

	EXPECT_EQ(computed_path_cost, 17);
	EXPECT_EQ(path_cost, computed_path_cost);
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
