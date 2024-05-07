// Implicit graph A* search
// Based on the Wikipedia pseudo-code, with some differences

#pragma once

#include <unordered_map>
#include <set>
#include <list>
#include <algorithm>
#include <limits>
#include <functional>
#include <queue>
#include <stack>
#include <utility>

#include <astar/detail/node.hpp>
#include <astar/cost_value.hpp>

namespace cds
{

namespace astar
{

/// Implicit graph A* search
/// @return The shortest path from the start node to the goal node
///			if one exists, otherwise, return an empty list.
template <	typename NodeType,
				typename ExpandFn, 
				typename CostFn,
				typename WeightFn,
				typename IsGoalFn,
				typename OutputIterator,
				typename HashFn = std::hash<NodeType> >
bool a_star_search(
	NodeType	start_node,
	ExpandFn	expand_fn,
	CostFn	cost_to_goal_fn,
	WeightFn	neighbor_weight_fn,
	IsGoalFn is_goal,
	OutputIterator out_it,
	cost_value_t<CostFn, NodeType>* opt_out_path_cost = nullptr,
	cost_value_t<CostFn, NodeType> max_cost = std::numeric_limits<cost_value_t<CostFn, NodeType>>::max())
{
	using cost_fn_t = 				cost_value_t<CostFn, NodeType>;
	using node_goal_cost_est_t =	detail_::node_goal_cost_estimate<NodeType, CostFn>;
	using node_info_t = 				detail_::node_info<NodeType, CostFn>;
	using node_collection_t =		std::unordered_map<NodeType, node_info_t, HashFn>;
	using detail_::NodeSetType;
																
	std::priority_queue<node_goal_cost_est_t> fringe;
	node_collection_t nodes;
	{
		typename node_collection_t::iterator start_node_it;
		tie(start_node_it, std::ignore) = 
			nodes.emplace(std::make_pair(start_node, node_info_t(NodeSetType::OPEN, 0.0)));
			
		fringe.emplace(node_goal_cost_est_t{&(*start_node_it), cost_to_goal_fn(start_node)});
	}

	std::list<NodeType> path;

	while (!fringe.empty())
	{
		auto min_cost_node = fringe.top();
		fringe.pop();

		// Might as well always assign this, even if we don't find a path
		if (opt_out_path_cost)
			*opt_out_path_cost = min_cost_node.cost;

		if (min_cost_node.cost > max_cost)
			return false; // We won't find a better solution

		NodeType const& n = min_cost_node.node_index->first;

		if (is_goal(n))
		{
			NodeType next = n;
			path.push_front(next);

			// Reconstruct path and return
			while (next != start_node)
			{
				auto n_it = nodes.find(next)->second;
				auto prev_node_it = n_it.prev_node;

				next = prev_node_it->first;

				path.push_front(next);
			}
			
			std::copy(path.begin(), path.end(), out_it);

			return true;
		}

		auto n_it = nodes.find(n);
		node_info_t& n_info = n_it->second;
		n_info.type = NodeSetType::CLOSED;

		auto neighbors = expand_fn(n);
		for (auto adj_node : neighbors)
		{
			auto adj_node_it = nodes.find(adj_node);
			if (adj_node_it != nodes.end() && adj_node_it->second.type == NodeSetType::CLOSED)
			{
				// Neighbor already evaluated
				continue;
			}

			// Distance from the starting node to a neighbor
			cost_fn_t const tentative_g_score = n_info.cost_to_node + neighbor_weight_fn(n, adj_node);
			cost_fn_t const f_score = tentative_g_score + cost_to_goal_fn(adj_node);

			if (adj_node_it == nodes.end())
			{
				// discover a new node
				tie(adj_node_it, std::ignore) = 
					nodes.emplace(std::make_pair(adj_node, node_info_t(NodeSetType::OPEN, tentative_g_score)));
			}
			else if (tentative_g_score >= adj_node_it->second.cost_to_node)
				continue;	// Sub-optimal path

			adj_node_it->second.prev_node = &(*n_it);
			adj_node_it->second.cost_to_node = tentative_g_score;

			fringe.emplace(node_goal_cost_est_t{&(*adj_node_it), f_score});
		}
	}

	// No path exists
	return false;
}

} // namespace astar

} // namespace cds
