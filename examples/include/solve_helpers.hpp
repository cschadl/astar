// Copyright (C) 2018 by Christopher Schadl <cschadl@gmail.com>

// Permission to use, copy, modify, and/or distribute this software for any purpose
// with or without fee is hereby granted.

// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD 
// TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS.
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
// DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
// WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
// ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#pragma once

#include <utility>
#include <vector>

#include <n_sq_puzzle.hpp>

namespace cds
{
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
	std::vector< n_sq_puzzle<N> > expand(const n_sq_puzzle<N>& p)
	{
		using Move = typename cds::n_sq_puzzle<N>::MoveType;

		std::vector< n_sq_puzzle<N> > next_states;

		std::array<Move, 4> moves = { Move::UP, Move::DOWN, Move::LEFT, Move::RIGHT };
		for (const Move& m : moves)
			if (p.can_move(m))
				next_states.push_back(p.moved(m));

		return next_states;
	}

	template <size_t Dim>
	void add_puzzle_state(std::vector<n_sq_puzzle<Dim>>& states, typename n_sq_puzzle<Dim>::state_t const& state)
	{
		states.push_back(n_sq_puzzle<Dim>());
		states.back().set(state);
	}
}

// hash function for n_sq_puzzle<N>
namespace std
{
	template<size_t N>
	class hash< cds::n_sq_puzzle<N> >
	{
	public:
		size_t operator()(cds::n_sq_puzzle<N> const& puz) const
		{
			hash<std::string> hash_fn;
			return hash_fn(puz.state_as_string());
		}
	};
}
