#pragma once

#include <vector>
#include <map>
#include <utility>
#include <algorithm>

using neighbor_weight_t = std::pair<char, int>;
using adj_list_graph_t = std::map<char, std::map<char, int>>;

inline std::vector<char> expand_adj_list_graph(adj_list_graph_t const& graph, char node)
{
	auto n_it = graph.find(node);
	if (n_it == graph.end())
		return std::vector<char>();

	auto const& neighbors = n_it->second;
	if (neighbors.empty())
		return std::vector<char>();

	std::vector<char> neighbor_nodes(neighbors.size());
	std::transform(neighbors.begin(), neighbors.end(),
		neighbor_nodes.begin(),
		[](neighbor_weight_t const& nw) { return nw.first; });

	return neighbor_nodes;
}

inline int neighbor_weight(adj_list_graph_t const& graph, char n, char m)
{
	// does not check that n, m are in graph (we'll throw an exception)
	auto const& n_neighbors_weights = graph.at(n);
	return n_neighbors_weights.at(m);
}
