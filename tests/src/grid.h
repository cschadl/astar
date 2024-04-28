#pragma once

#include <cmath>
#include <functional>

struct grid_node
{
	int x{0};
	int y{0};

	bool operator==(grid_node const& n) const { return this->x == n.x && this->y == n.y; }
	bool operator!=(grid_node const& n) const { return !(*this == n); }
};

namespace std
{

template <>
class hash<grid_node>
{
public:
	size_t operator()(const grid_node& node) const
	{
		constexpr unsigned int p1 = 73856093;
		constexpr unsigned int p2 = 83492791;

		int xi = std::floor(node.x);
		int yi = std::floor(node.y);

		return (xi * p1) ^ (yi * p2);
	}
};

}