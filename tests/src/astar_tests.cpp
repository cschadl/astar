#include <gtest/gtest.h>

#include <astar/a_star_search.hpp>
#include <n_sq_puzzle.hpp>

#include <map>
#include <vector>
#include <utility>
#include <algorithm>
#include <functional>
#include <utility>

using namespace cds;
namespace ph = std::placeholders;

namespace
{
	using neighbor_weight_t = std::pair<char, int>;
	using adj_list_graph_t = std::map<char, std::map<char, int>>;

	std::vector<char> expand_adj_list_graph(adj_list_graph_t const& graph, char node)
	{
		auto n_it = graph.find(node);
		if (n_it == graph.end())
			return std::vector<char>();

		auto const& neighbors = n_it->second;
		if (neighbors.empty())
			return std::vector<char>();

		std::vector<char> neighbor_nodes(neighbors.size());
		std::transform(neighbors.begin(), neighbors.end(),
			neighbor_nodes.begin(),
			[](neighbor_weight_t const& nw) { return nw.first; });

		return neighbor_nodes;
	}

	int neighbor_weight(adj_list_graph_t const& graph, char n, char m)
	{
		// does not check that n, m are in graph (we'll throw an exception)
		auto const& n_neighbors_weights = graph.at(n);
		return n_neighbors_weights.at(m);
	}

	int null_heuristic(char)
	{
		return 0;
	}

	bool is_goal(char n)
	{
		return n == 'z';
	}
}

TEST(AStarTest, FindShortestPathDijkstra)
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

	auto path = 
		astar::a_star_search(start_node, expand_fn, &null_heuristic, neighbor_weight_fn, &is_goal);

	ASSERT_EQ(path.size(), 5);
	
	auto p_it = path.begin();
	EXPECT_EQ(*(p_it++), 'a');
	EXPECT_EQ(*(p_it++), 'c');
	EXPECT_EQ(*(p_it++), 'd');
	EXPECT_EQ(*(p_it++), 'e');
	EXPECT_EQ(*(p_it++), 'z');

	auto path_cost = std::inner_product(
		path.begin(), std::prev(path.end()),
		std::next(path.begin()), 0, std::plus<>(), neighbor_weight_fn);

	EXPECT_EQ(path_cost, 17);
}