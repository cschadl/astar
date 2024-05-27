#include <gtest/gtest.h>

#include <n_sq_puzzle.hpp>
#include <solve_helpers.hpp>

#include <astar/a_star_search.hpp>
#include <astar/ida_star_search.hpp>

using namespace cds;

namespace
{
	template <size_t Dim>
	struct test_puzzle_wrapper { };

	template<>
	struct test_puzzle_wrapper<3>
	{
		static n_sq_puzzle<3> get_puzzle()
		{
			n_sq_puzzle<3> puzzle;
			puzzle.set({7, 2, 4, 3, 0, 1, 8, 5, 6});

			return puzzle;
		}

		static constexpr size_t expected_n_moves() { return 19; }
	};

	template<>
	struct test_puzzle_wrapper<4>
	{
		static n_sq_puzzle<4> get_puzzle()
		{
			n_sq_puzzle<4> puzzle;
			puzzle.set({ 12, 5, 7, 8, 1, 3, 11, 15, 9, 13, 6, 14, 2, 0, 4, 10 });

			return puzzle;
		}

		static constexpr size_t expected_n_moves() { return 45; }
	};

	template <size_t Dim>
	class NSqPuzzleSolver
	{
	protected:
		n_sq_puzzle<Dim> m_goal_state;

	public:
		static constexpr size_t puzzleDim() { return Dim; }

		NSqPuzzleSolver() = default;
		virtual ~NSqPuzzleSolver() { }

		bool is_goal(n_sq_puzzle<Dim> const& puz) const { return puz == m_goal_state; }

		int heuristic(n_sq_puzzle<Dim> const& puz) const { return tile_taxicab_dist(puz, m_goal_state); }

		static int dist(n_sq_puzzle<Dim> const&, n_sq_puzzle<Dim> const&) { return 1; }

		static std::vector<n_sq_puzzle<Dim>> expand(n_sq_puzzle<Dim> const& p)
		{
			return cds::expand<Dim>(p);
		}

		virtual bool solve(n_sq_puzzle<Dim> const& puzzle, std::vector<n_sq_puzzle<Dim>>& path) const = 0;
	};

	template <size_t Dim>
	class NSqPuzzleSolverAStar : public NSqPuzzleSolver<Dim>
	{
	public:
		NSqPuzzleSolverAStar() = default;

		bool solve(n_sq_puzzle<Dim> const& puzzle, std::vector<n_sq_puzzle<Dim>>& path) const override
		{
			return astar::a_star_search(
				puzzle,
				[this](auto const& n) { return this->expand(n); },
				[this](auto const& n) { return this->heuristic(n); },
				[this](auto const& n, auto const& m) { return this->dist(n, m); },
				[this](auto const& n) { return this->is_goal(n); },
				std::back_inserter(path));
		}
	};

	template <size_t Dim>
	class NSqPuzzleSolverIDAStar : public NSqPuzzleSolver<Dim>
	{
	public:
		NSqPuzzleSolverIDAStar() = default;

		bool solve(n_sq_puzzle<Dim> const& puzzle, std::vector<n_sq_puzzle<Dim>>& path) const override
		{
			return astar::ida_star_search(
				puzzle,
				[this](auto const& n) { return this->expand(n); },
				[this](auto const& n) { return this->heuristic(n); },
				[this](auto const& n, auto const& m) { return this->dist(n, m); },
				[this](auto const& n) { return this->is_goal(n); },
				std::back_inserter(path));
		}
	};
}

template <typename T>
class NSqPuzzleSolverTest : public testing::Test
{
protected:
	T theTest;

	static constexpr size_t dim() { return T::puzzleDim(); }
};

using NSqPuzzleSolverTestImplementations = 
	testing::Types<
		NSqPuzzleSolverAStar<3>, NSqPuzzleSolverAStar<4>,
		NSqPuzzleSolverIDAStar<3>, NSqPuzzleSolverIDAStar<4> >;

TYPED_TEST_SUITE(NSqPuzzleSolverTest, NSqPuzzleSolverTestImplementations);

TYPED_TEST(NSqPuzzleSolverTest, NSqPuzzleSolverTestImplementations)
{
	constexpr const size_t dim = TestFixture::dim();
	auto puzzle = test_puzzle_wrapper<dim>::get_puzzle();
	
	std::vector<n_sq_puzzle<dim>> path;
	bool found_path = this->theTest.solve(puzzle, path);

	ASSERT_TRUE(found_path);

	EXPECT_EQ(path.size(), test_puzzle_wrapper<dim>::expected_n_moves());
	EXPECT_EQ(path.front(), puzzle);
	EXPECT_TRUE(path.back().is_solved());

	using MoveType = typename n_sq_puzzle<dim>::MoveType;
	for (auto p_it = path.begin(); p_it != std::prev(path.end()); ++p_it)
	{
		bool found_move = false;

		for (auto move : { MoveType::UP, MoveType::DOWN, MoveType::LEFT, MoveType::RIGHT })
		{
			EXPECT_FALSE(*p_it == *std::next(p_it)) << *p_it;
			
			if (p_it->moved(move) == *std::next(p_it))
			{
				found_move = true;
				break;
			}
		}

		EXPECT_TRUE(found_move) << *p_it << " cannot be moved to " << *std::next(p_it);
	}
}
