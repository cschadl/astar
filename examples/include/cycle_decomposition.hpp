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

#include <vector>
#include <bitset>
#include <algorithm>

/// Cycle decomposition of v1 to v2
template <typename T, size_t N, typename OutputIterator>
bool cycle_decomposition(
	const std::array<T, N>& v1,
	const std::array<T, N>& v2,
	OutputIterator oi)
{
	std::bitset<N> visited;

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
		
		*oi++ = cycle;
	}

	return cycles_ok;
}

template <typename InputIterator, typename OutputIterator>
bool get_transpositions(InputIterator cycles_begin, InputIterator cycles_end, OutputIterator oi)
{
	for (auto cycle = cycles_begin; cycle != cycles_end; ++cycle)
	{
		auto cycle_start = cycle->begin();
		for (auto ci = std::next(cycle_start) ; ci != cycle->end() ; ++ci)
			*oi++ = std::make_pair(*cycle_start, *ci);
	}

	return true;
}
