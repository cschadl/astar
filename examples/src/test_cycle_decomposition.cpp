#include <iostream>
#include <array>
#include <iterator>
#include <random>
#include <algorithm>
#include <utility>

#include <cycle_decomposition.hpp>

int main(int argc, char** argv)
{
	//std::array<int, 5> a1 = { 1, 2, 3, 4, 5 };
	//std::array<int, 5> a2 = { 5, 4, 3, 2, 1 };
	//
	std::array<int, 8> a1 = { 1, 2, 3, 4, 5, 6, 7, 8 };
	//std::array<int, 8> a2 = { 8, 6, 5, 7, 3, 1, 2, 4 };
	std::array<int, 8> a2 = { 8, 6, 5, 7, 8, 1, 2, 4 };
	//
//	std::random_device rd;
//	std::mt19937 gen(rd());
//	std::array<int, 8> a2 = a1;
//	std::shuffle(a2.begin(), a2.end(), gen);

	std::cout << "a1: ( ";
	for (auto it = a1.begin(); it != a1.end(); ++it)
		std::cout << *it << (std::next(it) != a1.end() ? ", " : "");

	std::cout << " )" << std::endl;

	std::cout << "a2: ( ";
	for (auto it = a2.begin(); it != a2.end(); ++it)
		std::cout << *it << (std::next(it) != a2.end() ? ", " : "");

	std::cout << " )" << std::endl;

	std::vector< std::vector<int> > a1a2_cycles;
	if (!cycle_decomposition(a1, a2, std::back_inserter(a1a2_cycles)))
	{
		std::cout << "No cyclic decomposition found" << std::endl;

		return 0;
	}

//	size_t permutation_order = !a1a2_cycles.empty() ? 
//		std::accumulate(a1a2_cycles.begin(), a1a2_cycles.end(), 0,
//			[](size_t o, const std::vector<int>& c) { o+= (c.size() - 1); return o; })
//		: 0;
	std::vector< std::pair<int, int> > transpositions; 
	get_transpositions(a1a2_cycles.begin(), a1a2_cycles.end(), back_inserter(transpositions));

	size_t const permutation_order = transpositions.size();

	std::cout << "Cycle decomposition: (permutation order " << permutation_order << ")" << std::endl;

	for (const std::vector<int>& cycle : a1a2_cycles)
	{
		std::cout << "( ";
		for (auto c_it = cycle.begin(); c_it != cycle.end(); ++c_it)
		{
			std::cout << *c_it << (std::next(c_it) != cycle.end() ? " -> " : "");
		}

		std::cout << " )" << std::endl;
	}

	std::cout << "Transpositions: ";
	for (const auto& t : transpositions)
		std::cout << "(" << t.first << ", " << t.second << ")";
	std::cout << std::endl;

	return 0;
}
