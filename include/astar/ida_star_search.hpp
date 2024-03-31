#pragma once

#include <astar/detail/node.hpp>
#include <astar/detail/ida_search.hpp>
#include <utility>
#include <stack>
#include <list>

namespace cds
{

namespace astar
{

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
					 typename detail_::cost_fn_traits<CostFn, NodeType>::value max_cost = 
					 	detail_::cost_fn_traits<CostFn, NodeType>::max())
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
		std::tie(root_it, std::ignore) = 
			node_set.emplace(std::make_pair(start_node, node_info_t(detail_::NodeSetType::CLOSED, 0.0)));
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

}

}