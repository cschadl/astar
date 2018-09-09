#include <map>
#include <set>
#include <list>
#include <algorithm>
#include <limits>
#include <functional>

template <typename T, typename Val, typename Compare>
Val get_node_score_(const T& n, std::map<T, Val, Compare>& node_score_map)
{
	auto it = node_score_map.find(n);
	if (it == node_score_map.end())
	{
		tie(it, std::ignore) = node_score_map.insert(std::make_pair(n, std::numeric_limits<Val>::max()));
	}

	return it->second;
}

template <typename CostFn, typename NodeType>
struct cost_fn_traits_
{
	using value = decltype(std::declval<CostFn>()(std::declval<NodeType>(), std::declval<NodeType>()));
	
	static constexpr value max() { return std::numeric_limits<value>::max(); }
};

/// Implicit graph A* search
/// @return The shortest path from the start node to the goal node,
///			if one exists, otherwise, return an empty list.
template <	typename NodeType,
				typename ExpandFn, 
				typename CostFn,
				typename WeightFn,
				typename Compare = std::less<NodeType> >
std::list<NodeType> a_star_search(
	NodeType	start_node,
	NodeType goal_node,
	ExpandFn	expand_fn,
	CostFn	cost_fn,
	WeightFn	neighbor_weight_fn,
	typename cost_fn_traits_<CostFn, NodeType>::value max_cost = cost_fn_traits_<CostFn, NodeType>::max())
{
	using cost_fn_t = 			typename cost_fn_traits_<CostFn, NodeType>::value;
	using node_map_t =		 	std::map<NodeType, NodeType, Compare>;
	using path_score_map_t = 	std::map<NodeType, cost_fn_t, Compare>;
	using node_set_t = 			std::set<NodeType, Compare>;

	node_map_t	descendants;	// For each node, which node it can be most efficiently
										// reached from.  If a node can be reached from many nodes,
										// m_descendants will eventually contain the most efficient
										// previous step.
										// Used for path reconstruction
																
	node_set_t closed_set;

	node_set_t open_set;
	open_set.insert(start_node);

	// Total cost of getting from the start node to a given node
	path_score_map_t g_score;
	g_score[start_node] = 0;

	// Total cost estimate of getting from the start node to the
	// goal node by taking a path that passes throught a given node
	path_score_map_t f_score;
	f_score[start_node] = cost_fn(start_node, goal_node);

	std::list<NodeType> path;

	while (!open_set.empty())
	{
		// Get the node in the open set having the lowest f_score value
		// It would be good to use a bimap for this, but
		// for now, we'll just do a linear search
		auto min_f_score_node = std::min_element(open_set.begin(), open_set.end(),
			[&f_score](const NodeType& n1, const NodeType& n2)
			{
				return get_node_score_(n1, f_score) < get_node_score_(n2, f_score);
			});

		NodeType n = *min_f_score_node;

		if (n == goal_node)
		{
			path.push_front(n);

			// Reconstruct path and return
			while (n != start_node)
			{
				n = descendants[n];
				path.push_front(n);
			}
			
			return path;
		}

		open_set.erase(n);
		closed_set.insert(n);

		auto neighbors = expand_fn(n);
		for (auto adj_node : neighbors)
		{
			if (closed_set.find(adj_node) != closed_set.end())
			{
				// Neighbor already evaluated
				continue;
			}

			// Distance from the starting node to a neighbor
			cost_fn_t const tentative_g_score = get_node_score_(n, g_score) + neighbor_weight_fn(n, adj_node);
			cost_fn_t const f_score_ = tentative_g_score + cost_fn(adj_node, goal_node);

			if (f_score_ > max_cost)
				continue;

			if (open_set.find(adj_node) == open_set.end())
				open_set.insert(adj_node);	// Discover a new node
			else if (tentative_g_score >= get_node_score_(adj_node, g_score))
				continue; // Sub-optimal path

			descendants[adj_node] = n;
			g_score[adj_node] = tentative_g_score;
			f_score[adj_node] = f_score_;
		}
	}

	// No path exists
	return path;
}
