#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <functional>
#include <numeric>
#include <list>

#include "n_sq_puzzle.hpp"
#include "a_star_search.hpp"

using namespace std;

template <size_t N>
struct expand
{
	vector< n_sq_puzzle<N> > operator()(const n_sq_puzzle<N>& p) const
	{
		using Move = typename n_sq_puzzle<N>::MoveType;
	
		std::vector< n_sq_puzzle<N> > next_states;
	
		std::array<Move, 4> moves = { Move::UP, Move::DOWN, Move::LEFT, Move::RIGHT };
		for (const Move& m : moves)
			if (p.can_move(m))
				next_states.push_back(p.moved(m));
	
		return next_states;
	}
};

template <size_t N>
struct misplaced_tiles 
{
	size_t operator()(const n_sq_puzzle<N>& p, const n_sq_puzzle<N>& goal) const
	{
		auto const& p_state_array = p.get_state();
		auto const& goal_state_array = goal.get_state();
	
		size_t const diff = (N * N) - std::inner_product(
				p_state_array.begin(), p_state_array.end(),
				goal_state_array.begin(), 0,
				std::plus<>(), std::equal_to<>());
	
		return diff;
	}
};

//template <size_t N>
//struct tile_taxicab_dist
//{
//	size_t operator()(const n_sq_puzzle<N>& p, const n_sq_puzzle<N>& goal) const
//	{
//
//	}
//};

template <size_t N>
struct neighbor_dist 
{
	size_t operator()(const n_sq_puzzle<N>&, const n_sq_puzzle<N>&) const
	{
		return 1;
	}
};

template <size_t N>
struct no_max_cost
{
	bool operator()(const size_t)
	{
		return false;
	}
};

template <size_t N>
struct max_path_cost
{
	size_t max;

	bool operator()(const size_t)
	{
		return false;
	}
};

template <>
struct max_path_cost<3>
{
	bool operator()(const size_t cost)
	{
		return cost >= 31;
	}
};

template <>
struct max_path_cost<4>
{
	bool operator()(const size_t cost)
	{
		return cost >= 80;
	}
};

template <size_t N>
bool solve_n_sq_puzzle()
{
	using puzzle_t = n_sq_puzzle<N>;
	constexpr size_t Dim = puzzle_t::Dim;

	puzzle_t puz;
	puz.shuffle();

	puzzle_t puz_solved;

	std::cout << "Start puzzle state:" << endl << puz << endl << endl;
	std::cout << "Goal puzzle state:" << endl << puz_solved << endl << endl;

	auto solve_steps = 
		a_star_search(puz, puz_solved, expand<Dim>{}, misplaced_tiles<Dim>{}, neighbor_dist<Dim>{}, max_path_cost<Dim>{});

	if (solve_steps.empty())
	{
		cout << "Couldn't find path to goal" << endl;

		return false;
	}
	else
	{
		cout << "Found path (" << solve_steps.size() - 1 << " moves):" << endl;

		for (const puzzle_t& p : solve_steps)
		{
			cout << p << endl;
		}
	}

	return true;
}

int main(int argc, char** argv)
{
	size_t puzzle_dim = 3;
	if (argc > 1)
		puzzle_dim = atoi(argv[1]);

	switch (puzzle_dim)
	{
	case 2:
		solve_n_sq_puzzle<2>();
		break;
	case 3:
		solve_n_sq_puzzle<3>();
		break;
	case 4:
		solve_n_sq_puzzle<4>();
		break;
	default:
		std::cout << "Unsupported puzzle dimension " << puzzle_dim << endl;
	}

	return 0;
}
