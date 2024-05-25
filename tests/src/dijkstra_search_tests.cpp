#include <gtest/gtest.h>

#include "get_path_cost.h"

#include <astar/a_star_search.hpp>
#include <astar/ida_star_search.hpp>

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
