#include <gtest/gtest.h>

#include <n_sq_puzzle.hpp>

using namespace cds;
using MoveType = n_sq_puzzle<3>::MoveType;

TEST(NSqPuzzle, DefaultConstruction)
{
	n_sq_puzzle<3> puz;

	ASSERT_TRUE(puz.is_solved());
	EXPECT_EQ(puz.get_space_ij(), std::make_pair((size_t) 2, (size_t) 2));
}

TEST(NSqPuzzle, Move)
{
	n_sq_puzzle<3> puz;

	EXPECT_FALSE(puz.move(MoveType::RIGHT));
	EXPECT_TRUE(puz.move(MoveType::LEFT));
	EXPECT_TRUE(puz.move(MoveType::UP));

	EXPECT_EQ(puz.get_space_ij(), std::make_pair((size_t)1, (size_t) 1));

	EXPECT_TRUE(puz.can_move(MoveType::UP));
	EXPECT_TRUE(puz.can_move(MoveType::DOWN));
	EXPECT_TRUE(puz.can_move(MoveType::LEFT));
	EXPECT_TRUE(puz.can_move(MoveType::RIGHT));

	EXPECT_TRUE(puz.move(MoveType::UP));
	EXPECT_TRUE(puz.move(MoveType::LEFT));
	EXPECT_EQ(puz.get_space_ij(), std::make_pair((size_t) 0, (size_t) 0));

	std::array<MoveType, 2> j_move_types = { MoveType::RIGHT, MoveType::LEFT };
	
	for (size_t i = 0 ; i < 3 ; i++)
	{
		auto j_move = j_move_types[i % j_move_types.size()];

		if (j_move == MoveType::RIGHT)
		{
			for (size_t j = 1 ; j < 3 ;)
			{
				EXPECT_TRUE(puz.move(j_move));
				EXPECT_EQ(puz.get_space_ij(), std::make_pair(i, j++));
			}
		}
		else
		{
			for (int j = 1 ; j >=0 ;)
			{
				EXPECT_TRUE(puz.move(j_move));
				EXPECT_EQ(puz.get_space_ij(), std::make_pair(i, (size_t) (j--)));
			}
		}

		if ((i+1) < 3)
		{
			EXPECT_TRUE(puz.move(MoveType::DOWN));
		
			size_t expected_j = j_move == MoveType::RIGHT ? 2 : 0;
			EXPECT_EQ(puz.get_space_ij(), std::make_pair((i+1), expected_j));
		}
	}

	std::array<MoveType, 2> i_move_types = { MoveType::UP, MoveType::DOWN };

	for (int j = 2 ; j >=0 ; j--)
	{
		auto i_move = i_move_types[j % i_move_types.size()];

		if (i_move == MoveType::DOWN)
		{
			for (size_t i = 1 ; i < 3 ; )
			{
				EXPECT_TRUE(puz.move(i_move));
				EXPECT_EQ(puz.get_space_ij(), std::make_pair(i++, (size_t) j));
			}
		}
		else
		{
			for (int i = 1 ; i >= 0 ;)
			{
				EXPECT_TRUE(puz.move(i_move));
				EXPECT_EQ(puz.get_space_ij(), std::make_pair((size_t)(i--), (size_t) j));
			}
		}

		if (j > 0)
		{
			EXPECT_TRUE(puz.move(MoveType::LEFT));
			
			size_t expected_i = i_move == MoveType::UP ? 0 : 2;
			EXPECT_EQ(puz.get_space_ij(), std::make_pair(expected_i, (size_t)(j-1)));
		}
	}
}