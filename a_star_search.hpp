// Implicit graph A* search
// Based on the Wikipedia pseudo-code, with some differences

#pragma once

#include <map>
#include <set>
#include <list>
#include <algorithm>
#include <limits>
#include <functional>
#include <queue>
#include <utility>

namespace cds
{

namespace astar
{

namespace detail_
{

template <typename CostFn, typename NodeType>
struct cost_to_goal_traits
{
	using value = decltype(std::declval<CostFn>()(std::declval<NodeType>()));
	
	static constexpr value max() { return std::numeric_limits<value>::max(); }
};

enum class NodeSetType
{
	OPEN,
	CLOSED
};

template <typename NodeType, typename InfoType, typename Compare>
using node_map_iterator_t = typename std::map<NodeType, InfoType, Compare>::iterator;

template <typename NodeType, typename CostFn, typename Compare>
struct node_info
{
	using iterator_t = node_map_iterator_t<NodeType, node_info<NodeType, CostFn, Compare>, Compare>;

	NodeSetType type;
	iterator_t desc;
	typename cost_to_goal_traits<CostFn, NodeType>::value cost_to_node;

	node_info() = delete;

	node_info(NodeSetType type, double cost_to_node)
	: type(type)
	, cost_to_node(cost_to_node)
	{

	}
};

template <typename NodeType, typename CostFn, typename Compare>
struct node_goal_cost_estimate
{
	using index_t = typename node_info<NodeType, CostFn, Compare>::iterator_t;

	index_t	node_index;
	typename cost_to_goal_traits<CostFn, NodeType>::value cost;

	bool operator<(node_goal_cost_estimate const& rhs) const
	{
		return cost >= rhs.cost;	// min-priority queue, so this is flipped
	}
};

} // namespace detail_

/// Implicit graph A* search
/// @return The shortest path from the start node to the goal node
///			if one exists, otherwise, return an empty list.
template <	typename NodeType,
				typename ExpandFn, 
				typename CostFn,
				typename WeightFn,
				typename IsGoalFn,
				typename Compare = std::less<NodeType> >
std::list<NodeType> a_star_search(
	NodeType	start_node,
	ExpandFn	expand,
	CostFn	cost_to_goal,
	WeightFn	neighbor_weight,
	IsGoalFn is_goal,
	typename detail_::cost_to_goal_traits<CostFn, NodeType>::value max_cost = detail_::cost_to_goal_traits<CostFn, NodeType>::max())
{
	using cost_to_goal_t = 			typename detail_::cost_to_goal_traits<CostFn, NodeType>::value;
	using node_goal_cost_est_t =	detail_::node_goal_cost_estimate<NodeType, CostFn, Compare>;
	using node_info_t = 				detail_::node_info<NodeType, CostFn, Compare>;
	using node_collection_t =		std::map<NodeType, node_info_t, Compare>;
	using detail_::NodeSetType;
																
	std::priority_queue<node_goal_cost_est_t> fringe;
	node_collection_t nodes;
	{
		typename node_collection_t::iterator start_node_it;
		tie(start_node_it, std::ignore) = nodes.emplace(std::make_pair(start_node, node_info_t(NodeSetType::OPEN, 0.0)));
		fringe.emplace(node_goal_cost_est_t{start_node_it, cost_to_goal(start_node)});
	}

	std::list<NodeType> path;

	while (!fringe.empty())
	{
		auto min_cost_node = fringe.top();
		fringe.pop();

		if (min_cost_node.cost > max_cost)
			break; // We won't find a better solution

		NodeType n = min_cost_node.node_index->first;

		if (is_goal(n))
		{
			path.push_front(n);

			// Reconstruct path and return
			while (n != start_node)
			{
				auto n_it = nodes.find(n)->second;
				auto desc_it = n_it.desc;

				n = desc_it->first;

				path.push_front(n);
			}
			
			return path;
		}

		auto n_it = nodes.find(n);
		node_info_t& n_info = n_it->second;
		n_info.type = NodeSetType::CLOSED;

		auto neighbors = expand(n);
		for (auto adj_node : neighbors)
		{
			auto adj_node_it = nodes.find(adj_node);
			if (adj_node_it != nodes.end() && adj_node_it->second.type == NodeSetType::CLOSED)
			{
				// Neighbor already evaluated
				continue;
			}

			// Distance from the starting node to a neighbor
			cost_to_goal_t const tentative_g_score = n_info.cost_to_node + neighbor_weight(n, adj_node);
			cost_to_goal_t const f_score_ = tentative_g_score + cost_to_goal(adj_node);

			if (adj_node_it == nodes.end())
			{
				// discover a new node
				tie(adj_node_it, std::ignore) = nodes.emplace(std::make_pair(adj_node, node_info_t(NodeSetType::OPEN, tentative_g_score)));
			}
			else if (tentative_g_score >= adj_node_it->second.cost_to_node)
				continue;	// Sub-optimal path

			adj_node_it->second.desc = n_it;
			adj_node_it->second.cost_to_node = tentative_g_score;

			fringe.emplace(node_goal_cost_est_t{adj_node_it, f_score_});
		}
	}

	// No path exists
	return path;
}

} // namespace astar

} // namespace cds













