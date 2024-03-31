#pragma once

#include <utility>
#include <limits>

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

}

}