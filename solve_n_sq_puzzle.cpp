#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <functional>
#include <numeric>
#include <list>
#include <cstring>
#include <string>
#include <iterator>
#include <sstream>

#include "n_sq_puzzle.hpp"
#include "a_star_search.hpp"

using namespace std;
using namespace cds::astar;
using cds::n_sq_puzzle;
namespace ph = std::placeholders;

template <size_t N>
vector< n_sq_puzzle<N> > expand(const n_sq_puzzle<N>& p)
{
	using Move = typename n_sq_puzzle<N>::MoveType;

	std::vector< n_sq_puzzle<N> > next_states;

	std::array<Move, 4> moves = { Move::UP, Move::DOWN, Move::LEFT, Move::RIGHT };
	for (const Move& m : moves)
		if (p.can_move(m))
			next_states.push_back(p.moved(m));

	return next_states;
}

template <size_t N>
size_t misplaced_tiles(const n_sq_puzzle<N>& p, const n_sq_puzzle<N>& goal)
{
	auto const& p_state_array = p.get_state();
	auto const& goal_state_array = goal.get_state();

	size_t const diff = (N * N) - std::inner_product(
			p_state_array.begin(), p_state_array.end(),
			goal_state_array.begin(), 0,
			std::plus<>(), std::equal_to<>());

	return diff;
}

template <size_t N>
size_t tile_taxicab_dist(const n_sq_puzzle<N>& p, const n_sq_puzzle<N>& goal)
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

template <size_t N>
struct neighbor_dist
{
	size_t operator()(const n_sq_puzzle<N>&, const n_sq_puzzle<N>&)
	{
		return 1;
	}
};

enum class HeuristicType
{
	MISPLACED,	// # of misplaced tiles
	TAXICAB,		// distance between tiles in X and Y ("Manhattan" distance)
	ZERO			// null heuristic (always return 0)
};

struct puzzle_options
{
	size_t dim = 3;
	size_t max_cost = std::numeric_limits<size_t>::max();
	bool use_ida = false;
	std::vector<int> puzzle_state;

	HeuristicType heuristic_type = HeuristicType::TAXICAB;
};

// hash function for n_sq_puzzle<N>
namespace std
{
	// not sure how to specialize this as a template function
	template<>
	class hash< n_sq_puzzle<2> >
	{
	public:
		size_t operator()(n_sq_puzzle<2> const& puz) const
		{
			hash<std::string> hash_fn;
			return hash_fn(puz.state_as_string());
		}
	};

	template <>
	class hash< n_sq_puzzle<3> >
	{
	public:
		size_t operator()(n_sq_puzzle<3> const& puz) const
		{
			hash<std::string> hash_fn;
			return hash_fn(puz.state_as_string());
		}
	};

	template <>
	class hash< n_sq_puzzle<4> >
	{
	public:
		size_t operator()(n_sq_puzzle<4> const& puz) const
		{
			hash<std::string> hash_fn;
			return hash_fn(puz.state_as_string());
		}
	};
};

template <size_t N>
bool solve_n_sq_puzzle(puzzle_options const& options)
{
	using puzzle_t = n_sq_puzzle<N>;
	using state_t = typename puzzle_t::state_t;
	constexpr size_t Dim = puzzle_t::Dim;

	puzzle_t puz;
	if (!options.puzzle_state.empty())
	{
		// Make sure everything is kosher with the state that we passed in
		if (options.puzzle_state.size() != N*N)
		{
			std::cerr << "Invalid puzzle state dimension: " << options.puzzle_state.size() << endl;
			return false;
		}

		state_t puzzle_state;
		std::copy(options.puzzle_state.begin(), options.puzzle_state.end(), puzzle_state.begin());

		// set() validates the state (makes sure it is a permutation of the solved state)
		if (!puz.set(puzzle_state))
		{
			std::cerr << "Puzzle state is not a valid or solvable puzzle state" << endl;
			return false;
		}
	}
	else
	{
		// Solve a random puzzle configuration
		puz.shuffle();
	}

	puzzle_t puz_solved;

	std::cout << "Start puzzle state:" << endl << puz << endl << endl;
	std::cout << "Goal puzzle state:" << endl << puz_solved << endl << endl;

	std::list<puzzle_t> solve_steps;

	std::function<size_t(puzzle_t const&, puzzle_t const&)> h_fn;
	switch (options.heuristic_type)
	{
	case HeuristicType::MISPLACED:
		h_fn = &misplaced_tiles<N>;
		break;
	case HeuristicType::TAXICAB:
		h_fn = &tile_taxicab_dist<N>;
		break;
	case HeuristicType::ZERO:
		h_fn = [](puzzle_t const&, puzzle_t const&) { return 0; };
		break;
	}

	if (options.use_ida)
		tie(solve_steps, std::ignore) =
			ida_star_search(puz, puz_solved, &expand<Dim>, h_fn, neighbor_dist<Dim>{});
	else
		solve_steps = a_star_search(puz, puz_solved, &expand<Dim>, h_fn,neighbor_dist<Dim>{}, options.max_cost);

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

bool parse_cmd_line(int argc, char** argv, puzzle_options& options)
{
	for (int arg = 1 ; arg < argc ; arg++)
	{
		if (strcmp(argv[arg], "--dim") == 0)
		{
			if ((arg + 1) >= argc)
			{
				std::cerr << "Option requires argument: " << argv[arg] << endl;
				return false;
			}

			size_t dim = atoi(argv[++arg]);
			if (dim == 0)
				std::cerr << "Invalid puzzle dimension: " << argv[arg] << endl;

			options.dim = dim;
		}
		else if (strcmp(argv[arg], "--max_cost") == 0)
		{
			if ((arg + 1) >= argc)
			{
				std::cerr << "Option requires argument: " << argv[arg] << endl;
				return false;
			}

			size_t max_cost = atoi(argv[++arg]);
			if (max_cost == 0)
			{
				std::cerr << "Invalid max cost value: " << argv[arg] << endl;
				return false;

				options.max_cost = max_cost;
			}
		}
		else if (strcmp(argv[arg], "--ida") == 0)
		{
			options.use_ida = true;
		}
		else if (strcmp(argv[arg], "--state") == 0)
		{
			if ((arg + 1) >= argc)
			{
				std::cerr << "Option requires argument: " << argv[arg] << endl;
				return false;
			}

			// Split state input on spaces
			// We'll check the dimension later on
			std::string state_str(argv[++arg]);
			std::istringstream iss(state_str);
			std::vector<std::string> state(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());

			for (std::string const& val_str : state)
				options.puzzle_state.emplace_back(atoi(val_str.c_str()));
		}
		else if (strcmp(argv[arg], "--heuristic") == 0)
		{
			if ((arg + 1) >= argc)
			{
				std::cerr << "Option requires argument: " << argv[arg] << endl;
				return false;
			}

			std::string h_type_str(argv[++arg]);
			std::transform(h_type_str.begin(), h_type_str.end(), h_type_str.begin(), ::tolower);

			if (strcmp(h_type_str.c_str(), "misplaced") == 0)
				options.heuristic_type = HeuristicType::MISPLACED;
			else if (strcmp(h_type_str.c_str(), "taxicab") == 0)
				options.heuristic_type = HeuristicType::TAXICAB;
			else if (strcmp(h_type_str.c_str(), "zero") == 0)
				options.heuristic_type = HeuristicType::ZERO;
			else
			{
				std::cerr << "Unknown heuristic type: " << h_type_str << std::endl;
				return false;
			}
		}
		else
		{
			std::cout << "Unknown command line argument: " << argv[arg] << endl;
			return false;
		}
	}

	return true;
}

int main(int argc, char** argv)
{
	puzzle_options options;
	if (!parse_cmd_line(argc, argv, options))
		return 1;

	switch (options.dim)
	{
	case 2:
		solve_n_sq_puzzle<2>(options);
		break;
	case 3:
		solve_n_sq_puzzle<3>(options);
		break;
	case 4:
		solve_n_sq_puzzle<4>(options);
		break;
	default:
		std::cout << "Unsupported puzzle dimension " << options.dim << endl;
	}

	return 0;
}
