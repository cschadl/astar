#pragma once

#include <vector>
#include <bitset>
#include <algorithm>

/// Cycle decomposition of v1 to v2
template <typename T, size_t N>
std::vector< std::vector<T> > cycle_decomposition(
	const std::array<T, N>& v1,
	const std::array<T, N>& v2)
{
	std::bitset<N> visited;
	std::vector< std::vector<T> > cycles;

	bool cycles_ok = true;
	while (!visited.all() && cycles_ok)
	{
		size_t cycle_start = 0;
		// find the first unvisited bit
		while (cycle_start < visited.size() && visited.test(cycle_start))
			++cycle_start;

		std::vector<T> cycle;

		size_t c_idx = cycle_start;
		do
		{
			if (visited[c_idx])
			{
				cycles_ok = false;
				break;	// already been here (shoudln't happen)
			}

			visited[c_idx] = true;	// mark this visited
			cycle.push_back(v1[c_idx]);

			T to = v2[c_idx];

			// Find 'to' in v1
			auto it_to = std::find(v1.begin(), v1.end(), to);
			if (it_to == v1.end())
			{
				cycles_ok = false;
				break;	// nope
			}

			c_idx = std::distance(v1.begin(), it_to);
		} while (c_idx != cycle_start);
		
		if (cycle.size() > 1)
			cycles.push_back(cycle);
	}

	if (!cycles_ok)
		cycles.clear();

	return cycles;
}
