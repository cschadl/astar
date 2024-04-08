#pragma once

#include <astar/detail/node.hpp>
#include <astar/cost_value.hpp>

#include <stack>
#include <limits>
#include <unordered_map>

namespace cds
{

namespace astar
{

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
		cost_value_t<CostFn, NodeType> bound,
		cost_value_t<CostFn, NodeType> max_cost) -> std::pair<bool, cost_value_t<CostFn, NodeType>>
{
	using cost_t = cost_value_t<CostFn, NodeType>;
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

	cost_t min = std::numeric_limits<cost_value_t<CostFn, NodeType>>::max();

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

} // astar

} //cds