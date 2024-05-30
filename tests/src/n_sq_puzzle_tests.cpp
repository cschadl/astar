// Copyright (C) 2018 by Christopher Schadl <cschadl@gmail.com>

// Permission to use, copy, modify, and/or distribute this software for any purpose
// with or without fee is hereby granted.

// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD 
// TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS.
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
// DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
// WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
// ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#include <gtest/gtest.h>

#include <n_sq_puzzle.hpp>

using namespace cds;

template <typename T>
class NSqPuzzleTest : public testing::Test
{
public:
	T thePuzzle;
	static constexpr size_t dim() { return T::dim(); }

	using MoveType = typename T::MoveType;
};

using NSqPuzzleTestImplementations = 
	testing::Types<n_sq_puzzle<2>, n_sq_puzzle<3>, n_sq_puzzle<4>, n_sq_puzzle<5>>;

TYPED_TEST_SUITE(NSqPuzzleTest, NSqPuzzleTestImplementations);

TYPED_TEST(NSqPuzzleTest, DefaultConstructor)
{
	auto& puz = this->thePuzzle;
	ASSERT_TRUE(puz.is_solved());

	size_t ij = TestFixture::dim() - 1;
	EXPECT_EQ(puz.get_space_ij(), std::make_pair(ij, ij));
}

TYPED_TEST(NSqPuzzleTest, Move)
{
	TypeParam puz = this->thePuzzle;

	using MoveType = typename TestFixture::MoveType;

	EXPECT_FALSE(puz.move(MoveType::RIGHT));
	EXPECT_TRUE(puz.move(MoveType::LEFT));
	EXPECT_TRUE(puz.move(MoveType::UP));

	size_t ij_expected = TestFixture::dim() - 2;
	EXPECT_EQ(puz.get_space_ij(), std::make_pair(ij_expected, ij_expected));

	EXPECT_TRUE(TestFixture::dim() == 2 || puz.can_move(MoveType::UP));
	EXPECT_TRUE(puz.can_move(MoveType::DOWN));
	EXPECT_TRUE(TestFixture::dim() == 2 || puz.can_move(MoveType::LEFT));
	EXPECT_TRUE(puz.can_move(MoveType::RIGHT));

	while (puz.get_space_ij().first > 0)
		EXPECT_TRUE(puz.move(MoveType::UP));
	
	while (puz.get_space_ij().second > 0)
		EXPECT_TRUE(puz.move(MoveType::LEFT));
	
	EXPECT_EQ(puz.get_space_ij(), std::make_pair((size_t) 0, (size_t) 0));

	std::array<MoveType, 2> j_move_types = { MoveType::RIGHT, MoveType::LEFT };
	
	constexpr const size_t puzzle_dim = TestFixture::dim();

	// Move left->right, down, right->left, down, left->right ...
	for (size_t i = 0 ; i < puzzle_dim ; i++)
	{
		auto j_move = j_move_types[i % j_move_types.size()];

		if (j_move == MoveType::RIGHT)
		{
			for (size_t j = 1 ; j < puzzle_dim ;)
			{
				EXPECT_TRUE(puz.move(j_move));
				EXPECT_EQ(puz.get_space_ij(), std::make_pair(i, j++));
			}
		}
		else
		{
			for (int j = (puzzle_dim - 2) ; j >=0 ;)
			{
				EXPECT_TRUE(puz.move(j_move));
				EXPECT_EQ(puz.get_space_ij(), std::make_pair(i, (size_t) (j--)));
			}
		}

		if ((i+1) < puzzle_dim)
		{
			EXPECT_TRUE(puz.move(MoveType::DOWN));
		
			size_t expected_j = j_move == MoveType::RIGHT ? (puzzle_dim - 1) : 0;
			EXPECT_EQ(puz.get_space_ij(), std::make_pair((i+1), expected_j));
		}
	}

	// If the puzzle dimension is odd, the space will be in the lower right corner,
	// otherwise, it will be in the lower left corner.
	// Move it to the lower right corner
	while (puz.can_move(MoveType::RIGHT))
		puz.move(MoveType::RIGHT);

	ASSERT_EQ(puz.get_space_ij(), std::make_pair(puzzle_dim - 1, puzzle_dim - 1));

	std::array<MoveType, 2> i_move_types = { MoveType::UP, MoveType::DOWN };

	// Move bottom->top, left, top->bottom, left, bottom->top, left...
	int i_move_type = 0;
	for (int j = (puzzle_dim - 1) ; j >=0 ; j--)
	{
		auto i_move = i_move_types[i_move_type++ % i_move_types.size()];

		if (i_move == MoveType::DOWN)
		{
			for (size_t i = 1 ; i < puzzle_dim ; )
			{
				EXPECT_TRUE(puz.move(i_move));
				EXPECT_EQ(puz.get_space_ij(), std::make_pair(i++, (size_t) j));
			}
		}
		else
		{
			for (int i = (puzzle_dim - 2) ; i >= 0 ;)
			{
				EXPECT_TRUE(puz.move(i_move));
				EXPECT_EQ(puz.get_space_ij(), std::make_pair((size_t)(i--), (size_t) j));
			}
		}

		if (j > 0)
		{
			EXPECT_TRUE(puz.move(MoveType::LEFT));
			
			size_t expected_i = i_move == MoveType::UP ? 0 : (puzzle_dim - 1);
			EXPECT_EQ(puz.get_space_ij(), std::make_pair(expected_i, (size_t)(j-1)));
		}
	}
}
