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

#include <utility>
#include <limits>

#include <astar/cost_value.hpp>

namespace cds
{

namespace astar
{

namespace detail_
{

enum class NodeSetType
{
	OPEN,
	CLOSED
};

// would be nice if I could alias this to std::unordered_map<...>::value_type,
// but GCC gets pissy about that for some reason.
template <typename NodeType, typename InfoType>
using node_map_entry_ptr_t = typename std::pair<const NodeType, InfoType>*;

template <typename NodeType, typename CostFn>
struct node_info
{
	using entry_ptr_t = node_map_entry_ptr_t< NodeType, node_info<NodeType, CostFn> >;

	NodeSetType type;
	cost_value_t<CostFn, NodeType> cost_to_node;
	entry_ptr_t prev_node;	// pointer to previous node (for A* path reconstruction)

	node_info() = delete;

	node_info(NodeSetType type, cost_value_t<CostFn, NodeType> cost_to_node)
	: type(type)
	, cost_to_node(cost_to_node)
	, prev_node(nullptr)
	{

	}
};

template <typename NodeType, typename CostFn>
struct node_goal_cost_estimate
{
	typename node_info<NodeType, CostFn>::entry_ptr_t	node_index;
	cost_value_t<CostFn, NodeType> cost;

	bool operator<(node_goal_cost_estimate const& rhs) const
	{
		return cost >= rhs.cost;	// min-priority queue, so this is flipped
	}
};

} // namespace detail_

}

}