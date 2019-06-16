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

namespace cds
{

namespace astar
{

namespace detail_
{

template <typename CostFn, typename NodeType>
struct cost_fn_traits
{
	using value = decltype(std::declval<CostFn>()(std::declval<NodeType>()));
	
	static constexpr value max() { return std::numeric_limits<value>::max(); }
};

enum class NodeSetType
{
	OPEN,
	CLOSED
};

// would be nice if I could alias this to std::unordered_map<...>::value_type,
// but GCC gets pissy about that for some reason.
template <typename NodeType, typename InfoType>
using node_map_entry_t = typename std::pair<const NodeType, InfoType>*;

template <typename NodeType, typename CostFn>
struct node_info
{
	using entry_t = node_map_entry_t< NodeType, node_info<NodeType, CostFn> >;

	NodeSetType type;
	typename cost_fn_traits<CostFn, NodeType>::value cost_to_node;
	entry_t prev_node;	// pointer to previous node (for A* path reconstruction)

	node_info() = delete;

	node_info(NodeSetType type, typename cost_fn_traits<CostFn, NodeType>::value cost_to_node)
	: type(type)
	, cost_to_node(cost_to_node)
	, prev_node(nullptr)
	{

	}
};

template <typename NodeType, typename CostFn>
struct node_goal_cost_estimate
{
	typename node_info<NodeType, CostFn>::entry_t	node_index;
	typename cost_fn_traits<CostFn, NodeType>::value cost;

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
				typename HashFn = std::hash<NodeType> >
std::list<NodeType> a_star_search(
	NodeType	start_node,
	ExpandFn	expand_fn,
	CostFn	cost_to_goal_fn,
	WeightFn	neighbor_weight_fn,
	IsGoalFn is_goal,
	typename detail_::cost_fn_traits<CostFn, NodeType>::value max_cost = detail_::cost_fn_traits<CostFn, NodeType>::max())
{
	using cost_fn_t = 				typename detail_::cost_fn_traits<CostFn, NodeType>::value;
	using node_goal_cost_est_t =	detail_::node_goal_cost_estimate<NodeType, CostFn>;
	using node_info_t = 				detail_::node_info<NodeType, CostFn>;
	using node_collection_t =		std::unordered_map<NodeType, node_info_t, HashFn>;
	using detail_::NodeSetType;
																
	std::priority_queue<node_goal_cost_est_t> fringe;
	node_collection_t nodes;
	{
		typename node_collection_t::iterator start_node_it;
		tie(start_node_it, std::ignore) = nodes.emplace(std::make_pair(start_node, node_info_t(NodeSetType::OPEN, 0.0)));
		fringe.emplace(node_goal_cost_est_t{&(*start_node_it), cost_to_goal_fn(start_node)});
	}

	std::list<NodeType> path;

	while (!fringe.empty())
	{
		auto min_cost_node = fringe.top();
		fringe.pop();

		if (min_cost_node.cost > max_cost)
			break; // We won't find a better solution

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
			
			return path;
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
				tie(adj_node_it, std::ignore) = nodes.emplace(std::make_pair(adj_node, node_info_t(NodeSetType::OPEN, tentative_g_score)));
			}
			else if (tentative_g_score >= adj_node_it->second.cost_to_node)
				continue;	// Sub-optimal path

			adj_node_it->second.prev_node = &(*n_it);
			adj_node_it->second.cost_to_node = tentative_g_score;

			fringe.emplace(node_goal_cost_est_t{&(*adj_node_it), f_score});
		}
	}

	// No path exists
	return path;
}

namespace detail_
{

template <typename NodeType, typename CostFn, typename ExpandFn, typename NeighborWeightFn, typename IsGoalFn, typename HashFn>
auto ida_search(
		std::stack< typename node_info<NodeType, CostFn>::entry_t >& path,
		std::unordered_map<NodeType, node_info<NodeType, CostFn>, HashFn>& node_set,
		CostFn cost_to_goal_fn,
		ExpandFn expand,
		NeighborWeightFn neighbor_weight,
		IsGoalFn is_goal_fn,
		typename cost_fn_traits<CostFn, NodeType>::value bound,
		typename cost_fn_traits<CostFn, NodeType>::value max_cost) -> std::pair<bool, typename cost_fn_traits<CostFn, NodeType>::value>
{
	using cost_t = typename cost_fn_traits<CostFn, NodeType>::value;
	using node_info_t = node_info<NodeType, CostFn>;

	auto& node_it = path.top();

	NodeType const& node = node_it->first;
	node_info_t const& node_info = node_it->second;

	cost_t f = node_info.cost_to_node + cost_to_goal_fn(node);

	if (f > bound)
		return std::make_pair(false, f);

	if (f > max_cost)
		return std::make_pair(false, f);

	if (is_goal_fn(node))
		return std::make_pair(true, f);

	cost_t min = cost_fn_traits<CostFn, NodeType>::max();

	auto adj_nodes = expand(node);
	std::sort(adj_nodes.begin(), adj_nodes.end(),
		[&cost_to_goal_fn](NodeType const& n1, NodeType const& n2)
		{
			return cost_to_goal_fn(n1) < cost_to_goal_fn(n2);
		});

	for (NodeType const& adj_node : expand(node))
	{
		auto adj_node_it = node_set.find(adj_node);
		if (adj_node_it == node_set.end())
		{
			auto cost_to_adj_node = node_info.cost_to_node + neighbor_weight(node, adj_node);

			if (adj_node_it == node_set.end())
			{
				tie(adj_node_it, std::ignore) =
					node_set.emplace(std::make_pair(adj_node, node_info_t{ NodeSetType::CLOSED, cost_to_adj_node }));
			}

			path.push(&(*adj_node_it));

			// TODO - no more recursion
			std::pair<bool, cost_t> t =
					ida_search(
							path,
							node_set,
							cost_to_goal_fn, expand,
							neighbor_weight,
							is_goal_fn,
							bound,
							max_cost);

			if (t.first)
				return t;

			if (t.second < min)
				min = t.second;

			path.pop();

			// Node is no longer in the path, so remove it from the node set
			// We could also set the node type to OPEN, like we do for regular
			// A*, (and check for that when we expand) but the idea here
			// is to save memory at the cost of CPU usage...
			node_set.erase(adj_node_it);
		}
	}

	return std::make_pair(false, min);
}

} // detail_

template <	typename NodeType,
				typename ExpandFn,
				typename CostFn,
				typename WeightFn,
				typename IsGoalFn,
				typename HashFn = std::hash<NodeType>	>
std::pair<std::list<NodeType>, typename detail_::cost_fn_traits<CostFn, NodeType>::value>
ida_star_search(NodeType start_node,
					 ExpandFn expand,
					 CostFn cost_to_goal_fn,
					 WeightFn neighbor_weight_fn,
					 IsGoalFn is_goal_fn,
					 typename detail_::cost_fn_traits<CostFn, NodeType>::value max_cost = detail_::cost_fn_traits<CostFn, NodeType>::max())
{
	using cost_t = typename detail_::cost_fn_traits<CostFn, NodeType>::value;
	using node_info_t = detail_::node_info<NodeType, CostFn>;
	using node_set_t = std::unordered_map<NodeType, node_info_t, HashFn>;

	cost_t bound = cost_to_goal_fn(start_node);

	while (true)
	{
		node_set_t node_set;
		std::stack<typename node_info_t::entry_t> path_stack;

		typename node_set_t::iterator root_it;
		std::tie(root_it, std::ignore) = node_set.emplace(std::make_pair(start_node, node_info_t(detail_::NodeSetType::CLOSED, 0.0)));
		path_stack.push(&(*root_it));

		cost_t t = detail_::cost_fn_traits<CostFn, NodeType>::max();
		bool found = false;

		std::tie(found, t) = detail_::ida_search<NodeType, CostFn, ExpandFn, WeightFn, IsGoalFn, HashFn>(
				path_stack,
				node_set,
				cost_to_goal_fn, expand, neighbor_weight_fn,
				is_goal_fn, bound, max_cost);

		if (found)
		{
			std::list<NodeType> path;

			while (!path_stack.empty())
			{
				NodeType n = path_stack.top()->first;
				path.emplace_front(std::move(n));
				path_stack.pop();
			}

			return std::make_pair(path, bound);
		}

		if (t == detail_::cost_fn_traits<CostFn, NodeType>::max())
			break;	// No path exists

		bound = t;

		if (bound > max_cost)
			break;
	}

	return std::make_pair(std::list<NodeType>(), bound);
}

} // namespace astar

} // namespace cds













