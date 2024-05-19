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
#include <optional>

#include <n_sq_puzzle.hpp>
#include <solve_helpers.hpp>
#include <astar/a_star_search.hpp>
#include <astar/ida_star_search.hpp>

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
	std::optional<size_t> shuffle_seed;

	HeuristicType heuristic_type = HeuristicType::TAXICAB;
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
		puz.shuffle(options.shuffle_seed);
	}

	puzzle_t puz_solved;

	std::cout << "Start puzzle state:" << endl << puz << endl << endl;
	std::cout << "Goal puzzle state:" << endl << puz_solved << endl << endl;

	std::list<puzzle_t> solve_steps;

	std::function<size_t(puzzle_t const&)> h_fn;
	switch (options.heuristic_type)
	{
	case HeuristicType::MISPLACED:
		h_fn = [&puz_solved](puzzle_t const& puz) { return misplaced_tiles<N>(puz, puz_solved); };
		break;
	case HeuristicType::TAXICAB:
		h_fn = [&puz_solved](puzzle_t const& puz) { return tile_taxicab_dist(puz, puz_solved); };
		break;
	case HeuristicType::ZERO:
		h_fn = [](puzzle_t const&) { return 0; };
		break;
	}

	auto goal_fn = [](puzzle_t const& p) { return p.is_solved(); };

	bool success = false;
	if (options.use_ida)
	{
		success = ida_star_search(
			puz, &expand<Dim>, h_fn, neighbor_dist<Dim>{}, goal_fn,
			std::back_inserter(solve_steps), nullptr, options.max_cost);
	}
	else
	{
		success = a_star_search(
			puz, &expand<Dim>, h_fn, neighbor_dist<Dim>{}, goal_fn,
			std::back_inserter(solve_steps), nullptr, options.max_cost);
	}
	
	if (!success)
	{
		cout << "Couldn't find path to goal" << endl;
	}
	else
	{
		cout << "Found path (" << solve_steps.size() - 1 << " moves):" << endl;

		for (const puzzle_t& p : solve_steps)
		{
			cout << p << endl;
		}
	}

	return success;
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
			if (max_cost <= 0)
			{
				std::cerr << "Invalid max cost value: " << argv[arg] << endl;
				return false;
			}

			options.max_cost = max_cost;
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
		else if (strcmp(argv[arg], "--seed") == 0)
		{
			try
			{
				options.shuffle_seed = std::stoul(argv[++arg]);
			}
			catch (std::invalid_argument&)
			{
				std::cout << "Error parsing seed" << std::endl;
				return false;
			}
			catch (std::out_of_range)
			{
				std::cout << "specified seed is too large" << std::endl;
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
