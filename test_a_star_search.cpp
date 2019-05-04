#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <numeric>

#include "a_star_search.hpp"

using namespace std;
using namespace cds::astar;

namespace
{
	const int GRID_WIDTH = 45;
	const int GRID_HEIGHT = 45;
};

struct node
{
	int x;
	int y;
	int index;
	
	node(int x_, int y_, int index_)
		: x(x_), y(y_), index(index_)
	{
	}

	node()
		: x(0), y(0), index(0)
	{
	}

	node(const node&) = default;
	node& operator=(const node&) = default;
	node(node&&) = default;
	node& operator=(node&&) = default;

	bool operator<(const node& rhs) const { return index < rhs.index; }
	bool operator==(const node& rhs) const { return x == rhs.x && y == rhs.y; }
	bool operator!=(const node& rhs) const { return !(*this == rhs); }

	friend ostream& operator<<(ostream& os, const node& n);
};

ostream& operator<<(ostream& os, const node& n)
{
	os << "( " << n.x << ", " << n.y << " )";
	return os;
}

bool expansion_ok(const node& n)
{
	// Configuration space is a 45 x 45 grid with a "T" shaped obstacle

	if (n.x < 0 || n.x >= GRID_WIDTH)
		return false;

	if (n.y < 0 || n.y >= GRID_HEIGHT)
		return false;

	if (n.x >= 15 && n.x <= 27 && n.y <= 35)
		return false;

	if (n.x >= 7 && n.x <= 38 && n.y >= 28 && n.y <= 35)
		return false;

	return true;
}

int grid_index(int x, int y)
{
	return x + y * GRID_HEIGHT;
}

std::vector<node> expand(node n)
{
	//std::cout << "Expanding around node " << n << endl; 
	vector<node> adj_nodes;

	vector<int> adj_xy = { -1, 0, 1 };
	for (int dx : adj_xy)
	{
		for (int dy : adj_xy)
		{
//			if (abs(dx) == abs(dy))
//				continue;	// no diagonal moves
			if (dx == 0 && dy == 0)
				continue;

			int node_x = n.x + dx;
			int node_y = n.y + dy;

			node adj_node(node_x, node_y, grid_index(node_x, node_y));
			if (expansion_ok(adj_node))
			{
				//cout << "Expanding " << adj_node << endl;
				adj_nodes.push_back(adj_node);
			}
		}
	}
	
	return adj_nodes;
}

double node_dist(const node& n1, const node& n2)
{
	double const dx = (double) n1.x - (double) n2.x;
	double const dy = (double) n1.y - (double) n2.y;

	//return abs(dx) + abs(dy);
	return sqrt((dx*dx) + (dy*dy));
}

double zero_heuristic(const node&, const node&)
{
	return 0.0;
}

int main(int argc, char** argv)
{
	if (argc > 1 && argc < 5)
	{
		std::cout << "Usage: " << argv[0] << " start_x start_y goal_x goal_y <-zero>" << std::endl;
		return 0;
	}

	int start_x = argc > 2 ? std::atoi(argv[1]) : 10;
	int start_y = argc > 3 ? std::atoi(argv[2]) : 0;
	int goal_x = argc > 4 ? std::atoi(argv[3]) : 32;
	int goal_y = argc > 5 ? std::atoi(argv[4]) : 12;

	auto h_fn = &node_dist;
	if (argc >= 6 && strcmp(argv[5], "-zero") == 0)
		h_fn = &zero_heuristic;

	node start_node(start_x, start_y, grid_index(start_x, start_y));
	node goal_node(goal_x, goal_y, grid_index(goal_x, goal_y));

	list<node> path;
	double cost;
	tie(path, cost) = ida_star_search(start_node, goal_node, &expand, h_fn, &node_dist);

	if (path.empty())
		cout << "Couldn't find path to goal" << endl;
	else
	{
		double const total_cost = 
			std::inner_product(
					path.begin(), std::prev(path.end()), 
					std::next(path.begin()), 
					0.0, std::plus<double>(), &node_dist);

		cout 	<< "Optimal path from " << start_node << " to " << goal_node 
				<< " (cost " << total_cost << "):" << endl << endl;

		for (auto pn_it = path.begin() ; pn_it != path.end() ; ++pn_it)
		{
			const node& pn = *pn_it;
			cout << pn << (std::next(pn_it) != path.end() ? " -> " : "");
		}

		cout << endl;
	}

	return 0;
}
