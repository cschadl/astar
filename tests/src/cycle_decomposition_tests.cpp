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

#include <cycle_decomposition.hpp>

#include <iterator>
#include <array>
#include <vector>

TEST(CycleDecomposition, PermutationOddCycles)
{
	std::array<int, 9> a1 = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
	std::array<int, 9> a2 = { 4, 6, 9, 8, 3, 1, 7, 2, 5 };

	std::vector< std::vector<int> > decompositions;
	ASSERT_TRUE(cycle_decomposition(a1, a2, std::back_inserter(decompositions)));
	EXPECT_EQ(decompositions.size(), 3);
		
	auto cycle_it = decompositions.begin();
	EXPECT_EQ(*(cycle_it++), std::vector<int>({1, 4, 8, 2, 6}));
	EXPECT_EQ(*(cycle_it++), std::vector<int>({3, 9, 5}));
	EXPECT_EQ(*(cycle_it++), std::vector<int>({7}));
}

TEST(CycleDecomposition, PermutationEvenCycles)
{
	std::array<int, 6> a1 = { 1, 2, 3, 4, 5, 6 };
	std::array<int, 6> a2 = { 4, 1, 6, 2, 3, 5 };

	std::vector< std::vector<int> > decompositions;
	ASSERT_TRUE(cycle_decomposition(a1, a2, std::back_inserter(decompositions)));
	EXPECT_EQ(decompositions.size(), 2);

	auto cycle_it = decompositions.begin();
	EXPECT_EQ(*(cycle_it++), std::vector<int>({1, 4, 2}));
	EXPECT_EQ(*(cycle_it++), std::vector<int>({3, 6, 5}));
}

TEST(CycleDecomposition, Disjoint)
{
	std::array<int, 8> a1 = { 1, 2, 3, 4, 5, 6, 7, 8};
	std::array<int, 8> a2 = { 1, 2, 3, 4, 5, 6, 7, 8};

	std::vector< std::vector<int> > decomposition;
	ASSERT_TRUE(cycle_decomposition(a1, a2, std::back_inserter(decomposition)));
	EXPECT_EQ(decomposition.size(), 8);

	auto i = 1;
	for (auto const& a : decomposition)
	{
		EXPECT_EQ(a.size(), 1);
		EXPECT_EQ(a.front(), i++);
	}
}

TEST(CycleDecomposition, CompleteCycle)
{
	std::array<int, 5> a1 = { 1, 2, 3, 4, 5 };
	std::array<int, 5> a2 = { 4, 5, 1, 2, 3 };

	std::vector< std::vector<int> > decomposition;
	ASSERT_TRUE(cycle_decomposition(a1, a2, std::back_inserter(decomposition)));
	EXPECT_EQ(decomposition.size(), 1);
	EXPECT_EQ(decomposition.front(), std::vector<int>({ 1, 4, 2, 5, 3 }));
}