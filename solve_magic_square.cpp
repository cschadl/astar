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

#include "a_star_search.hpp"

using namespace std;
using namespace cds::astar;

using magic_square_t = std::vector< std::vector<int> >;

// expand
vector<magic_square_t> expand(magic_square_t const& sq)
{
	static std::array<int, 9> ms_numbers = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };

	vector<magic_square_t> next_states;

	// Replace a non-distinct number in sq with a number from ms_numbers
	std::array<int, 9> sq_vals;
	for (size_t i = 0 ; i < 3 ; i++)
		for (size_t j = 0 ; j < 3; j++)
			sq_vals[3 * j + i] = sq[i][j];

	sort(sq_vals.begin(), sq_vals.end());

//	std::vector<int> missing;
//	std::set_difference(
//			ms_numbers.begin(), ms_numbers.end(), 
//			sq_vals.begin(), sq_vals.end(),
//			back_inserter(missing));

	vector<int> replace;

	auto it = std::adjacent_find(sq_vals.begin(), sq_vals.end()) ;
	while (it != sq_vals.end())
	{
		replace.push_back(*it);
		it = std::adjacent_find(next(it), sq_vals.end());
	}

	for (int r : replace)
	{
		for (int i = 0; i < 3 ; i++)
		{
			for (int j = 0; j < 3 ; j++)
			{
				if (sq[i][j] == r)
				{
					for (int m : ms_numbers)
					{
						magic_square_t sq_next = sq;
						sq_next[i][j] = m;

						next_states.push_back(sq_next);
					}
				}
			}
		}
	}

	return next_states;
}

// cost
int cost_fn(magic_square_t const& sq)
{
	std::vector<int> sq_unique(9);
	for (size_t i = 0 ; i < 3 ; i++)
		for (size_t j = 0 ; j < 3; j++)
			sq_unique[3 * j + i] = sq[i][j];

	sort(sq_unique.begin(), sq_unique.end());

	sq_unique.erase(unique(sq_unique.begin(), sq_unique.end()), sq_unique.end());

	int const misplaced = 9 - sq_unique.size();
	return misplaced;
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
	std::list<magic_square_t> states = 
		a_star_search(s, &expand, &cost_fn, &n_sq_diff, &is_magic_square);

	if (states.empty())
		return -1;

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

