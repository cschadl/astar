// Copyright (C) 2018 by Christopher Schadl <cschadl@gmail.com>

// Permission to use, copy, modify, and/or distribute this software for any purpose
// with or without fee is hereby granted.

// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD 
// TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS.
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
// DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
// WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
// ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#pragma once

#include <astar/detail/node.hpp>
#include <astar/detail/ida_search.hpp>
#include <astar/cost_value.hpp>
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
				typename OutputIterator,
				typename HashFn = std::hash<NodeType>	>
bool ida_star_search(
	NodeType start_node,
	ExpandFn expand,
	CostFn cost_to_goal_fn,
	WeightFn neighbor_weight_fn,
	IsGoalFn is_goal_fn,
	OutputIterator out_it,
	cost_value_t<CostFn, NodeType>* opt_out_path_cost = nullptr,
	cost_value_t<CostFn, NodeType> max_cost = std::numeric_limits<cost_value_t<CostFn, NodeType>>::max())
{
	using cost_t = cost_value_t<CostFn, NodeType>;
	using node_info_t = detail_::node_info<NodeType, CostFn>;
	using node_set_t = std::unordered_map<NodeType, node_info_t, HashFn>;

	cost_t bound = cost_to_goal_fn(start_node);

	while (true)
	{
		node_set_t node_set;
		std::stack<typename node_info_t::entry_ptr_t> path_stack;

		typename node_set_t::iterator root_it;
		std::tie(root_it, std::ignore) = 
			node_set.emplace(std::make_pair(start_node, node_info_t(detail_::NodeSetType::CLOSED, 0.0)));
		path_stack.push(&(*root_it));

		cost_t t = std::numeric_limits<cost_value_t<CostFn, NodeType>>::max();
		bool found = false;

		std::tie(found, t) = detail_::ida_search<NodeType, CostFn, ExpandFn, WeightFn, IsGoalFn, HashFn>(
				path_stack,
				node_set,
				cost_to_goal_fn, expand, neighbor_weight_fn,
				is_goal_fn, bound, max_cost);

		if (opt_out_path_cost)
			*opt_out_path_cost = bound;

		if (found)
		{
			std::list<NodeType> path;

			while (!path_stack.empty())
			{
				NodeType n = path_stack.top()->first;
				path.emplace_front(std::move(n));
				path_stack.pop();
			}

			std::copy(path.begin(), path.end(), out_it);

			return true;
		}

		if (t == std::numeric_limits<cost_value_t<CostFn, NodeType>>::max())
			break;	// No path exists

		bound = t;

		if (bound > max_cost)
			break;
	}

	return false;
}

}

}