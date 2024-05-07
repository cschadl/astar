#include <vector>
#include <iostream>
#include <string>
#include <array>
#include <limits>
#include <numeric>
#include <algorithm>
#include <fstream>
#include <functional>
#include <sstream>

#include <astar/a_star_search.hpp>

using namespace std;
using namespace cds::astar;

using magic_square_t = std::vector< std::vector<int> >;

namespace
{

static std::vector<magic_square_t> canonical_magic_square = {
		{ { 8, 1, 6 }, { 3, 5, 7 }, { 4, 9, 2 } },
		{ { 6, 1, 8 }, { 7, 5, 3 }, { 2, 9, 4 } },
		{ { 4, 9, 2 }, { 3, 5, 7 }, { 8, 1, 6 } },
		{ { 2, 5, 4 }, { 7, 5, 3 }, { 6, 1, 8 } },
		{ { 8, 3, 4 }, { 1, 5, 9 }, { 6, 7, 2 } },
		{ { 4, 3, 8 }, { 9, 5, 1 }, { 2, 7, 6 } },
		{ { 6, 7, 2 }, { 1, 5, 9 }, { 8, 3, 4 } },
		{ { 2, 7, 6 }, { 9, 5, 1 }, { 4, 3, 8 } }
};

}

// "basic" expand
// Just generate successor states by replacing each entry in sq
// with a different number.  Note that this will generate
// lots and lots of successor states!
vector<magic_square_t> expand(magic_square_t const& sq)
{
	static std::array<int, 9> ms_numbers = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };

	vector<magic_square_t> next_states;

	for (size_t i = 0 ; i < 3 ; i++)
	{
		for (size_t j = 0 ; j < 3 ; j++)
		{
			for (size_t k = 0 ; k < 9 ; k++)
			{
				if (sq[i][j] != ms_numbers[k])
				{
					magic_square_t sq_n = sq;
					sq_n[i][j] = ms_numbers[k];

					next_states.emplace_back(std::move(sq_n));
				}
			}
		}
	}

	return next_states;
}

vector<magic_square_t> expand2(magic_square_t const& sq)
{
	vector<magic_square_t> next_states;

	// Generate successor states where each entry in sq
	// is transformed into a canonical magic square entry
	for (size_t i = 0 ; i < 3 ; i++)
	{
		for (size_t j = 0 ; j < 3; j++)
		{
			for (magic_square_t const& c_sq : canonical_magic_square)
			{
				if (c_sq[i][j] != sq[i][j])
				{
					magic_square_t n_sq = sq;
					n_sq[i][j] = c_sq[i][j];

					next_states.emplace_back(std::move(n_sq));
				}
			}
		}
	}

	return next_states;
}

// weight
int n_sq_diff(magic_square_t const& s1, magic_square_t const& s2)
{
	int weight = 0;

	for (int i = 0; i < 3; i++)
		for (int j = 0 ; j < 3 ; j++)
			weight+= abs(s1[i][j] - s2[i][j]);

	return weight;
}

// heuristic function
// compares sq with each of the "canonical" magic squares,
// returns the one with the smallest diff
int cost_fn(magic_square_t const& sq)
{
	auto min_val = std::numeric_limits<int>::max();
	for (magic_square_t const& canonical_sq : canonical_magic_square)
	{
		int const dist = n_sq_diff(sq, canonical_sq);
		if (dist < min_val)
			min_val = dist;
	}

	return min_val;
}

namespace std
{

template<>
class hash<magic_square_t>
{
public:
	size_t operator()(magic_square_t const& sq) const
	{
		std::ostringstream oss;
		for (size_t i = 0 ; i < 3 ; i++)
			for (size_t j = 0 ; j < 3 ; j++)
				oss << sq[i][j];

		hash<string> str_hash;
		return str_hash(oss.str());
	}
};

}

// is_goal
bool is_magic_square(magic_square_t const& sq)
{
	std::vector<int> sq_unique(9);
	for (size_t i = 0 ; i < 3 ; i++)
		for (size_t j = 0 ; j < 3; j++)
			sq_unique[3 * j + i] = sq[i][j];

	sort(sq_unique.begin(), sq_unique.end());

	if (std::unique(sq_unique.begin(), sq_unique.end()) != sq_unique.end())
		return false;

	for (int i = 0 ; i < 3 ; i++)
	{
		const auto& row_i = sq[i];
		int row_i_sum = std::accumulate(row_i.begin(), row_i.end(), 0);
		if (row_i_sum != 15)
			return false;
	}

	for (int j = 0 ; j < 3; j++)
	{
		int col_j_sum = 0;

		for (int i = 0 ; i < 3 ; i++)
			col_j_sum += sq[i][j];

		if (col_j_sum != 15)
			return false;
	}

	size_t diag_sum1 = 0;
	for (int i = 0 ; i < 3 ; i++)
		diag_sum1 += sq[i][i];

	if (diag_sum1 != 15)
		return false;

	size_t const diag_sum2 = sq[0][2] + sq[1][1] + sq[2][0];
	if (diag_sum2 != 15)
		return false;

	return true;
}

int formingMagicSquare(magic_square_t s)
{
	std::vector<magic_square_t> states;
	int path_cost;

	bool success = a_star_search(
		s, &expand2, &cost_fn, &n_sq_diff, &is_magic_square,
		std::back_inserter(states), &path_cost);

	if (!success)
		return false;

	std::cout << std::endl;
	for (auto it = states.begin() ; it != states.end() ; ++it)
	{
		for (int i = 0 ; i < 3 ; i++)
		{
			for (int j = 0 ; j < 3; j++)
				std::cout << (*it)[i][j] << " ";

			std::cout << endl;
		}
		
		std::cout << endl;
	}

	return std::inner_product(
			states.begin(), prev(states.end()) , 
			next(states.begin()), 
			0, std::plus<int>(), &n_sq_diff);
}

int main()
{
	 //ofstream fout(getenv("OUTPUT_PATH"));

    vector<vector<int>> s(3);
    for (int i = 0; i < 3; i++) {
        s[i].resize(3);

        for (int j = 0; j < 3; j++) {
            cin >> s[i][j];
        }

        cin.ignore(numeric_limits<streamsize>::max(), '\n');
    }

    int result = formingMagicSquare(s);

	 std::cout << "Min cost is " << result << endl;

    //fout.close();

    return 0;
}

