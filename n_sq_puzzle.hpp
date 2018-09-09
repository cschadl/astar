#include <algorithm>
#include <array>
#include <random>
#include <tuple>
#include <utility>
#include <iostream>
#include <iomanip>
#include <sstream>

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
class n_sq_puzzle
{
private:
	std::array<int, N * N>	m_state;

	size_t m_space_index;	// Index of empty space in puzzle (0 in m_state)

	std::pair<size_t, size_t> row_col_from_index(size_t idx) const
	{
		size_t i = idx / N;
		size_t j = idx % N;

		return std::make_pair(i, j);
	}

public:
	static constexpr size_t Dim = N;

	/// Creates a n_sq_puzzle in the solved configuration.
	/// Use shuffle() to shuffle the puzzle state to a random configuration.
	n_sq_puzzle()
		: m_state(create_index_array<int, N * N>())
	{
		std::rotate(m_state.begin(), m_state.begin() + 1, m_state.end());
		m_space_index = N * N - 1;
	}

	static constexpr size_t size() { return N; }

	int& operator()(size_t i, size_t j) { return m_state[N * i + j]; }
	const int& operator()(size_t i, size_t j) const { return m_state[N * i + j]; }

	std::pair<size_t, size_t> get_space_ij() const { return row_col_from_index(m_space_index); }

	const std::array<int, N * N>& get_state() const { return m_state; }

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

	bool operator<(const n_sq_puzzle<N>& rhs) const
	{
		// No, sir, I don't like it
		std::stringstream ss1;
		for (int i : m_state)
			ss1 << i;

		unsigned long long state;
		ss1 >> state;

		std::stringstream ss2;
		for (int i : rhs.m_state)
			ss2 << i;

		unsigned long long rhs_state;
		ss2 >> rhs_state;

		return state < rhs_state;
	}

	bool is_solved() const
	{
		return *this == n_sq_puzzle<N>();
	}

	void shuffle()
	{
		std::random_device rd;
		shuffle(rd());
	}

	void shuffle(unsigned int seed)
	{
		// if the empty space is in the lower right hand corner,
		// the puzzle is solvable iff the permutation of the
		// remaining pieces is even.
		//
		// Generate a random configuration with the 0 in the last place
		// test if the non-zero elements are an even permuation of
		// the solved state
		// If they are, move the 0 to some other random space
		// Unfortunately, it's not trivial to obtain the order
		// of a permuattion, so I'm just gonna make a whole bunch
		// of random moves for now
		std::mt19937 gen(seed);
		std::uniform_int_distribution<short> distr(0, 3);
		for (size_t i = 0 ; i < 9999 ; i++)
		{
			MoveType m = static_cast<MoveType>(distr(gen));
			move(m);
		}
	}

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

	template <size_t M>
	friend std::ostream& operator<<(std::ostream& os, const n_sq_puzzle<M>& puz);
};

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

template <size_t M>
std::ostream& operator<<(std::ostream& os, const n_sq_puzzle<M>& puz)
{
	constexpr size_t digits = num_digits<M * M - 1>();

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
