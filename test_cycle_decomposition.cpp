#include <iostream>
#include <array>
#include <iterator>

#include "cycle_decomposition.hpp"

int main(int argc, char** argv)
{
	//std::array<int, 5> a1 = { 1, 2, 3, 4, 5 };
	//std::array<int, 5> a2 = { 5, 4, 3, 2, 1 };
	//
	std::array<int, 7> a1 = { 0, 1, 2, 3, 4, 5, 6 };
	std::array<int, 7> a2 = { 2, 5, 3, 0, 1, 6, 4 };

	std::vector< std::vector<int> > a1a2_cycles =
		cycle_decomposition(a1, a2);

	for (const std::vector<int>& cycle : a1a2_cycles)
	{
		std::cout << "( ";
		for (auto c_it = cycle.begin(); c_it != cycle.end(); ++c_it)
		{
			std::cout << *c_it << (std::next(c_it) != cycle.end() ? " -> " : "");
		}

		std::cout << " )" << std::endl;
	}

	return 0;
}
