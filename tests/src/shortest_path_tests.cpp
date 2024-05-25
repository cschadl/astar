#include <gtest/gtest.h>

#include <astar/a_star_search.hpp>
#include <astar/ida_star_search.hpp>

#include <vector>
#include <unordered_set>
#include <limits>
#include <cmath>

#include "get_path_cost.h"

using namespace cds;

namespace
{
	struct grid_node
	{
		int x{0};
		int y{0};

		bool operator==(grid_node const& n) const { return this->x == n.x && this->y == n.y; }
		bool operator!=(grid_node const& n) const { return !(*this == n); }
	};

}

namespace std
{

template <>
class hash<grid_node>
{
public:
	size_t operator()(const grid_node& node) const
	{
		constexpr unsigned int p1 = 73856093;
		constexpr unsigned int p2 = 83492791;

		int xi = std::floor(node.x);
		int yi = std::floor(node.y);

		return (xi * p1) ^ (yi * p2);
	}
};

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

	std::unordered_set<grid_node> theObstacles = {
		grid_node{0, 2}, grid_node{ 0, 3 },
		grid_node{1, 2},
		grid_node{3, 2},
		grid_node{4, 0}, grid_node{4, 2},
		grid_node{5, 0}, grid_node{5, 1},
		grid_node{6, 0}, grid_node{6, 1},
		grid_node{7, 0}, grid_node{7, 1}
	};

	class GridSearchTest
	{
	protected:
		std::unordered_set<grid_node> m_grid_obstacles;
		grid_node m_goal_node;

		std::vector<grid_node> expand(grid_node const& n) const
		{
			return expand_grid(m_grid_obstacles, grid_node{0, 0}, grid_node{7, 7}, n);
		}

		double heuristic(grid_node const& n) const
		{
			return node_dist(n, m_goal_node);
		}

		bool is_goal(grid_node const& n) const
		{
			return n == m_goal_node;
		}

	public:
		GridSearchTest(
			std::unordered_set<grid_node> const& obstacles, grid_node const& goal_node)
			: m_grid_obstacles(obstacles)
			, m_goal_node(goal_node)
		{

		}

		virtual bool doSearch(
			grid_node const& start_node,
			std::vector<grid_node>& out_path,
			double& path_cost) = 0;

		grid_node const& goal_node() const { return m_goal_node; }
	};

	class AStarGridSearchTest : public GridSearchTest
	{
	public:
		AStarGridSearchTest()
			: GridSearchTest(theObstacles, grid_node{7, 3})
		{

		}

		bool doSearch(
			grid_node const& start_node,
			std::vector<grid_node>& out_path, 
			double& path_cost) override
		{
			return astar::a_star_search(
				start_node,
				[this](grid_node const& n) { return expand(n); },
				[this](grid_node const& n) { return heuristic(n); },
				node_dist,
				[this](grid_node const& n) { return is_goal(n); },
				std::back_inserter(out_path), &path_cost);
		}
	};

	class IDAStarGridSearchTest : public GridSearchTest
	{
	public:
		IDAStarGridSearchTest()
			: GridSearchTest(theObstacles, grid_node{7, 3})
		{

		}

		bool doSearch(
			grid_node const& start_node,
			std::vector<grid_node>& out_path, 
			double& path_cost) override
		{
			return astar::ida_star_search(
				start_node,
				[this](grid_node const& n) { return expand(n); },
				[this](grid_node const& n) { return heuristic(n); },
				node_dist,
				[this](grid_node const& n) { return is_goal(n); },
				std::back_inserter(out_path), &path_cost);
		}
	};
}

template <typename T>
class GridSearchShortestPathTest : public testing::Test
{
protected:
	T theTest;
};

using GridSearchShortestPathTestImplementations =
	testing::Types<AStarGridSearchTest, IDAStarGridSearchTest>;

TYPED_TEST_SUITE(GridSearchShortestPathTest, GridSearchShortestPathTestImplementations);

TYPED_TEST(GridSearchShortestPathTest, GridSearchShortestPathTestImplementations)
{
	grid_node const start_node{0, 0};

	std::vector<grid_node> path;
	double path_cost;

	bool found_path = this->theTest.doSearch(start_node, path, path_cost);

	ASSERT_TRUE(found_path);
	ASSERT_FALSE(path.empty());

	double computed_path_cost = get_path_cost(path.begin(), path.end(), node_dist);

	EXPECT_EQ(path.size(), 8);
	EXPECT_NEAR(computed_path_cost, sqrt(2) * 3 + 4, std::numeric_limits<double>::epsilon() * 100);
	EXPECT_EQ(computed_path_cost, path_cost);

	EXPECT_EQ(path.front(), start_node);
	EXPECT_EQ(path.back(), this->theTest.goal_node());
	
	EXPECT_TRUE(std::none_of(path.begin(), path.end(), 
		[](grid_node const& n) { return theObstacles.find(n) != theObstacles.end(); }));

	EXPECT_TRUE(std::all_of(path.begin(), path.end(),
		[](grid_node const& n) { return n.x >= 0 && n.y >= 0 && n.x <= 7 && n.y <= 7; }));
}
