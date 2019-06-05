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
	using value = decltype(std::declval<CostFn>()(std::declval<NodeType>(), std::declval<NodeType>()));
	
	static constexpr value max() { return std::numeric_limits<value>::max(); }
};

enum class NodeSetType
{
	OPEN,
	CLOSED
};

template <typename NodeType, typename InfoType, typename Compare>
using node_map_iterator_t = typename std::pair<const NodeType, InfoType> *;

template <typename NodeType, typename CostFn, typename Compare>
struct node_info
{
	using iterator_t = node_map_iterator_t<NodeType, node_info<NodeType, CostFn, Compare>, Compare>;

	NodeSetType type;
	typename cost_fn_traits<CostFn, NodeType>::value cost_to_node;
	iterator_t desc;

	node_info() = delete;

	node_info(NodeSetType type, double cost_to_node)
	: type(type)
	, cost_to_node(cost_to_node)
	, desc(nullptr)
	{

	}
};

template <typename NodeType, typename CostFn, typename Compare>
struct node_goal_cost_estimate
{
	using index_t = typename node_info<NodeType, CostFn, Compare>::iterator_t;

	index_t	node_index;
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
				typename Compare = std::hash<NodeType> >
std::list<NodeType> a_star_search(
	NodeType	start_node,
	NodeType goal_node,
	ExpandFn	expand_fn,
	CostFn	cost_fn,
	WeightFn	neighbor_weight_fn,
	typename detail_::cost_fn_traits<CostFn, NodeType>::value max_cost = detail_::cost_fn_traits<CostFn, NodeType>::max())
{
	using cost_fn_t = 				typename detail_::cost_fn_traits<CostFn, NodeType>::value;
	using node_goal_cost_est_t =	detail_::node_goal_cost_estimate<NodeType, CostFn, Compare>;
	using node_info_t = 				detail_::node_info<NodeType, CostFn, Compare>;
	using node_collection_t =		std::unordered_map<NodeType, node_info_t, Compare>;
	using detail_::NodeSetType;
																
	std::priority_queue<node_goal_cost_est_t> fringe;
	node_collection_t nodes;
	{
		typename node_collection_t::iterator start_node_it;
		tie(start_node_it, std::ignore) = nodes.emplace(std::make_pair(start_node, node_info_t(NodeSetType::OPEN, 0.0)));
		fringe.emplace(node_goal_cost_est_t{&(*start_node_it), cost_fn(start_node, goal_node)});
	}

	std::list<NodeType> path;

	while (!fringe.empty())
	{
		auto min_cost_node = fringe.top();
		fringe.pop();

		if (min_cost_node.cost > max_cost)
			break; // We won't find a better solution

		NodeType const& n = min_cost_node.node_index->first;

		if (n == goal_node)
		{
			NodeType next = n;
			path.push_front(next);

			// Reconstruct path and return
			while (next != start_node)
			{
				auto n_it = nodes.find(next)->second;
				auto desc_it = n_it.desc;

				next = desc_it->first;

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
			cost_fn_t const f_score = tentative_g_score + cost_fn(adj_node, goal_node);

			if (adj_node_it == nodes.end())
			{
				// discover a new node
				tie(adj_node_it, std::ignore) = nodes.emplace(std::make_pair(adj_node, node_info_t(NodeSetType::OPEN, tentative_g_score)));
			}
			else if (tentative_g_score >= adj_node_it->second.cost_to_node)
				continue;	// Sub-optimal path

			adj_node_it->second.desc = &(*n_it);
			adj_node_it->second.cost_to_node = tentative_g_score;

			fringe.emplace(node_goal_cost_est_t{&(*adj_node_it), f_score});
		}
	}

	// No path exists
	return path;
}

namespace detail_
{

template <typename NodeType, typename CostFn, typename ExpandFn, typename NeighborWeightFn, typename Compare>
auto ida_search(
		std::stack< node_map_iterator_t<NodeType, node_info<NodeType, CostFn, Compare>, Compare> >& path,
		std::unordered_map<NodeType, node_info<NodeType, CostFn, Compare>, Compare>& node_set,
		CostFn cost_fn,
		ExpandFn expand,
		NeighborWeightFn neighbor_weight,
		NodeType goal_node,
		typename cost_fn_traits<CostFn, NodeType>::value cost_to_current_node,
		typename cost_fn_traits<CostFn, NodeType>::value bound) -> std::pair<bool, typename cost_fn_traits<CostFn, NodeType>::value>
{
	using cost_t = typename cost_fn_traits<CostFn, NodeType>::value;
	using node_info_t = node_info<NodeType, CostFn, Compare>;

	auto& node_it = path.top();
	NodeType const& node = node_it->first;

	cost_t f = cost_to_current_node + cost_fn(node, goal_node);

	if (f > bound)
		return std::make_pair(false, f);

	if (node == goal_node)
		return std::make_pair(true, f);

	cost_t min = cost_fn_traits<CostFn, NodeType>::max();

	auto adj_nodes = expand(node);
	std::sort(adj_nodes.begin(), adj_nodes.end(),
		[&cost_fn, &goal_node](NodeType const& n1, NodeType const& n2)
		{
			return cost_fn(n1, goal_node) < cost_fn(n2, goal_node);
		});

	for (NodeType const& adj_node : expand(node))
	{
		auto adj_node_it = node_set.find(adj_node);
		if (adj_node_it == node_set.end() || adj_node_it->second.type == NodeSetType::OPEN)
		{
			if (adj_node_it == node_set.end())
				tie(adj_node_it, std::ignore) = node_set.insert(std::make_pair(adj_node, node_info_t{ NodeSetType::CLOSED, 0.0 }));

			path.push(&(*adj_node_it));

			// TODO - no more recursion
			std::pair<bool, cost_t> t =
					ida_search<NodeType, CostFn, ExpandFn, NeighborWeightFn, Compare>(
							path,
							node_set,
							cost_fn, expand,
							neighbor_weight,
							goal_node,
							cost_to_current_node + neighbor_weight(node, adj_node),
							bound);

			if (t.first)
				return t;

			if (t.second < min)
				min = t.second;

			path.pop();
			adj_node_it->second.type = NodeSetType::OPEN;	// Mark this node as longer in path
		}
	}

	return std::make_pair(false, min);
}

} // detail

template <	typename NodeType,
				typename ExpandFn,
				typename CostFn,
				typename WeightFn,
				typename Compare = std::hash<NodeType>	>
std::pair<std::list<NodeType>, typename detail_::cost_fn_traits<CostFn, NodeType>::value>
ida_star_search(NodeType start_node,
					 NodeType goal_node,
					 ExpandFn expand,
					 CostFn cost_fn,
					 WeightFn neighbor_weight_fn)
{
	using cost_t = typename detail_::cost_fn_traits<CostFn, NodeType>::value;
	using node_info_t = detail_::node_info<NodeType, CostFn, Compare>;
	using node_set_t = std::unordered_map<NodeType, node_info_t, Compare>;

	cost_t bound = cost_fn(start_node, goal_node);

	node_set_t node_set;
	std::stack<typename node_info_t::iterator_t> path_stack;

	typename node_set_t::iterator root_it;
	std::tie(root_it, std::ignore) = node_set.insert(std::make_pair(start_node, node_info_t(detail_::NodeSetType::OPEN, 0.0)));
	path_stack.push(&(*root_it));

	while (true)
	{
		cost_t t = detail_::cost_fn_traits<CostFn, NodeType>::max();
		bool found = false;

		std::tie(found, t) = detail_::ida_search<NodeType, CostFn, ExpandFn, WeightFn, Compare>(
				path_stack,
				node_set,
				cost_fn, expand, neighbor_weight_fn,
				goal_node, 0.0, bound);

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
	}

	return std::make_pair(std::list<NodeType>(), bound);
}

} // namespace astar

} // namespace cds






































