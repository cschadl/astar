#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <functional>
#include <numeric>
#include <list>
#include <cstring>

#include "n_sq_puzzle.hpp"
#include "a_star_search.hpp"

using namespace std;
using namespace cds::astar;

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

template <size_t N>
struct tile_taxicab_dist
{
	size_t operator()(const n_sq_puzzle<N>& p, const n_sq_puzzle<N>& goal) const
	{
		size_t taxicab_sum = 0;

		// start at 1, since we don't want to include the empty space
		for (size_t i = 1 ; i < (N*N) ; i++)
		{
			int i_p, j_p;
			std::tie(i_p, j_p) = p.get_ij_of(i);

			int i_goal, j_goal;
			std::tie(i_goal, j_goal) = goal.get_ij_of(i);

			size_t const taxicab_x = std::abs(i_goal - i_p);
			size_t const taxicab_y = std::abs(j_goal - j_p);

			taxicab_sum += (taxicab_x + taxicab_y);
		}

		return taxicab_sum;
	}
};

template <size_t N>
struct null_heuristic
{
	size_t operator()(const n_sq_puzzle<N>&, const n_sq_puzzle<N>&) const
	{
		return 0;
	}
};

template <size_t N>
struct neighbor_dist 
{
	size_t operator()(const n_sq_puzzle<N>&, const n_sq_puzzle<N>&) const
	{
		return 1;
	}
};

template <size_t N>
bool solve_n_sq_puzzle(size_t max_cost, bool use_ida_search)
{
	using puzzle_t = n_sq_puzzle<N>;
	constexpr size_t Dim = puzzle_t::Dim;

	puzzle_t puz;
	puz.shuffle();

	puzzle_t puz_solved;

	std::cout << "Start puzzle state:" << endl << puz << endl << endl;
	std::cout << "Goal puzzle state:" << endl << puz_solved << endl << endl;

	std::list<puzzle_t> solve_steps;

	if (use_ida_search)
		tie(solve_steps, std::ignore) =
			ida_star_search(puz, puz_solved, expand<Dim>{}, tile_taxicab_dist<Dim>{}, neighbor_dist<Dim>{});
	else
		solve_steps = a_star_search(puz, puz_solved, expand<Dim>{}, tile_taxicab_dist<Dim>{},neighbor_dist<Dim>{}, max_cost);

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

	size_t max_cost = std::numeric_limits<size_t>::max();

	if (argc > 2)
		max_cost = atoi(argv[2]);
	if (max_cost == 0)
	{
		std::cerr << "Invalid max cost value: " << argv[2] << endl;
		return 1;
	}

	bool use_ida = false;
	if (argc > 3 && ::strcmp(argv[3], "-ida") == 0)
		use_ida = true;

	switch (puzzle_dim)
	{
	case 2:
		solve_n_sq_puzzle<2>(max_cost, use_ida);
		break;
	case 3:
		solve_n_sq_puzzle<3>(max_cost, use_ida);
		break;
	case 4:
		solve_n_sq_puzzle<4>(max_cost, use_ida);
		break;
	default:
		std::cout << "Unsupported puzzle dimension " << puzzle_dim << endl;
	}

	return 0;
}
