// N-square puzzle (8-puzzle, 16-puzzle, etc.)

#pragma once

#include <algorithm>
#include <array>
#include <random>
#include <tuple>
#include <utility>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "cycle_decomposition.hpp"

namespace cds
{

namespace n_sq_puz_detail_
{

template <typename T, size_t N, size_t... I>
auto create_index_array_impl(std::index_sequence<I...>)
{
	return std::array<T, N>{ {I...} };
}

template <typename T, size_t N>
auto create_index_array()
{
	return create_index_array_impl<T, N>(std::make_index_sequence<N>{});
}

template <size_t N>
constexpr size_t num_digits()
{
	if (N == 0)
		return 1;

	size_t num = N;
	size_t digits = 0;
	while (num)
	{
		num /= 10;
		digits++;
	}

	return digits;
}

} // n_sq_puz_detail

template <size_t N>
class n_sq_puzzle
{
	static_assert(N > 1, "Invalid puzzle dimension");
public:
	using state_t = std::array<int, N*N>;

private:
	state_t	m_state;

	size_t m_space_index;	// Index of empty space in puzzle (0 in m_state)

	std::pair<size_t, size_t> row_col_from_index(size_t idx) const
	{
		size_t i = idx / N;
		size_t j = idx % N;

		return std::make_pair(i, j);
	}

	void move_space_to_lower_right_()
	{
		size_t space_i, space_j;
		std::tie(space_i, space_j) = get_space_ij();

		for (size_t mv_down = 0 ; mv_down < (N - 1) - space_i ; mv_down++)
			move(MoveType::DOWN);
		for (size_t mv_right = 0 ; mv_right < (N - 1) - space_j ; mv_right++)
			move(MoveType::RIGHT);
	}

	// return true if state is an even permutation of this state
	bool is_even_permutation_of_(state_t const& state) const
	{
		std::vector< std::vector<int> > state_cycle_decomp;
		if (!cycle_decomposition(m_state, state, std::back_inserter(state_cycle_decomp)))
			return false;	// state is not a permutation of m_state

		size_t const permutation_order =
			std::accumulate(state_cycle_decomp.begin(), state_cycle_decomp.end(), 0,
				[](size_t o, const std::vector<int>& cycle)
				{
					o += (cycle.size() - 1);

					return o;
				});

		return (permutation_order % 2 == 0);
	}

public:
	static constexpr size_t Dim = N;

	static constexpr size_t dim() { return N; }

	/// Creates a n_sq_puzzle in the solved configuration.
	/// Use shuffle() to shuffle the puzzle state to a random configuration.
	n_sq_puzzle()
		: m_state(n_sq_puz_detail_::create_index_array<int, N * N>())
	{
		std::rotate(m_state.begin(), m_state.begin() + 1, m_state.end());
		m_space_index = N * N - 1;
	}

	bool set(state_t const& state)
	{
		auto space_it = std::find(state.begin(), state.end(), 0);
		if (space_it == state.end())
			return false;

		size_t space_index = std::distance(state.begin(), space_it);

		// First, move the empty space (0 element) to the lower right corner
		n_sq_puzzle<N> test_puz;
		test_puz.m_state = state;
		test_puz.m_space_index = space_index;

		test_puz.move_space_to_lower_right_();

		bool const is_even_permutation = is_even_permutation_of_(test_puz.m_state);

		if (is_even_permutation)
		{
			m_state = state;
			m_space_index = space_index;
		}

		return is_even_permutation;
	}

	static constexpr size_t size() { return N; }

	int& operator()(size_t i, size_t j) { return m_state[N * i + j]; }
	const int& operator()(size_t i, size_t j) const { return m_state[N * i + j]; }

	std::pair<size_t, size_t> get_space_ij() const { return row_col_from_index(m_space_index); }

	std::pair<size_t, size_t> get_ij_of(int item) const
	{
		auto item_it = std::find(m_state.begin(), m_state.end(), item);
		if (item_it == m_state.end())
			std::terminate();	// whatever

		size_t const index = std::distance(m_state.begin(), item_it);
		return row_col_from_index(index);
	}

	const state_t& get_state() const { return m_state; }

	bool operator==(const std::array< std::array<int, N>, N> & rhs) const
	{
		for (size_t i = 0 ; i < N ; i++)
			for (size_t j = 0 ; j < N ; j++)
				if ((*this)(i, j) != rhs[i][j])
					return false;

		return true;
	}

	bool operator==(const n_sq_puzzle<N>& rhs) const
	{
		return m_state == rhs.m_state;
	}

	bool operator!=(const n_sq_puzzle<N>& rhs) const
	{
		return !((*this) == rhs);
	}

	std::string state_as_string() const
	{
		std::stringstream ss;
		for (auto i : m_state)
			ss << i;

		return ss.str();
	}

	bool operator<(const n_sq_puzzle<N>& rhs) const
	{
		return this->state_as_string() < rhs.state_as_string();
	}

	bool is_solved() const
	{
		return *this == n_sq_puzzle<N>();
	}

	bool shuffle(std::optional<unsigned int> seed = std::nullopt)
	{
		if (!seed.has_value())
		{
			std::random_device rd;
			return shuffle_([&rd] { return rd(); });
		}

		return shuffle_([s = seed.value()] { return s; });
	}

protected:
	template <typename SeedFn>
	bool shuffle_(SeedFn seed_fn)
	{
		// If the empty space is in the lower right hand corner,
		// the puzzle is solvable iff the permutation of the
		// remaining pieces is even.
		//
		// Generate a random configuration with the 0 in the last place,
		// test if the non-zero elements are an even permuation of the puzzle state.
		// If they are, move the 0 to some other random space
		
		// First, move the empty space (0 element) to the lower right corner
		move_space_to_lower_right_();

		if (m_space_index != N*N - 1)
			throw std::runtime_error("Error moving empty space for permutation configuration!");

		auto shuffled_state = n_sq_puzzle<N>().m_state;

		bool is_even_permutation = false;
		while (!is_even_permutation)
		{
			std::mt19937 gen(seed_fn());
			std::shuffle(shuffled_state.begin(), shuffled_state.end() - 1, gen);
			if (shuffled_state == m_state)
				continue;

			is_even_permutation = is_even_permutation_of_(shuffled_state);
		}

		m_state = shuffled_state;
		m_space_index = N*N - 1;

		// Finally, move the space index to a random position 
		std::mt19937 gen_ij(seed_fn());
		std::uniform_int_distribution<int> random_ij(0, N - 1);
		size_t space_i = random_ij(gen_ij);
		size_t space_j = random_ij(gen_ij);

		for (size_t mv_up = 0; mv_up < (N - 1) - space_i ; mv_up++)
			move(MoveType::UP);
		for (size_t mv_left = 0; mv_left < (N - 1) - space_j ; mv_left++)
			move(MoveType::LEFT);

#ifdef DEBUG
		if (get_space_ij() != std::make_pair(space_i, space_j))
			throw std::runtime_error("Space i, j mismatch!");
#endif

		return true;
	}

public:
	enum class MoveType : short
	{
		UP,
		DOWN,
		LEFT,
		RIGHT
	};

	bool can_move(MoveType mt) const
	{
		size_t i, j;
		std::tie(i, j) = row_col_from_index(m_space_index);

		switch (mt)
		{
			case MoveType::UP:
				return i > 0;
			case MoveType::DOWN:
				return i < (N - 1);
			case MoveType::LEFT:
				return j > 0;
			case MoveType::RIGHT:
				return j < (N - 1);
		}

		// shoudln't happen
		return false;
	}

	/// Move the empty space
	/// This isn't the most intuitive way of doing things, but I'm 
	/// mostly interested in expanding states for a A* search.
	bool move(MoveType mt)
	{
		if (!can_move(mt))
			return false;

		size_t i,j;
		std::tie(i,j) = row_col_from_index(m_space_index);

		switch (mt)
		{
		case MoveType::UP:
			std::swap((*this)(i, j), (*this)(i-1, j));
			m_space_index = N * (i - 1) + j;
			break;
		case MoveType::DOWN:
			std::swap((*this)(i, j), (*this)(i+1, j));
			m_space_index = N * (i+1) + j;
			break;
		case MoveType::LEFT:
			std::swap((*this)(i, j), (*this)(i, j-1));
			m_space_index = N * i + (j - 1);
			break;
		case MoveType::RIGHT:
			std::swap((*this)(i, j), (*this)(i, j+1));
			m_space_index = N * i + (j + 1);
			break;
		}

		return true;
	}

	n_sq_puzzle<N> moved(MoveType m) const
	{
		n_sq_puzzle<N> mp(*this);
		mp.move(m);

		return mp;
	}

	template <size_t M>
	friend std::ostream& operator<<(std::ostream& os, const n_sq_puzzle<M>& puz);
};

template <size_t M>
std::ostream& operator<<(std::ostream& os, const n_sq_puzzle<M>& puz)
{
	constexpr size_t digits = n_sq_puz_detail_::num_digits<M * M - 1>();

	for (size_t i = 0 ; i < M ; i++)
	{
		os << "[ ";
		for (size_t j = 0 ; j < M ; j++)
		{
			int const p = puz(i, j);
			os << std::setw(digits)
				<< (p > 0 ? std::to_string(p) : std::string(" ")) << (j < (M-1) ? " " : "");
		}
		os << " ]" << std::endl;
	}

	return os;
}

} // namespace cds
