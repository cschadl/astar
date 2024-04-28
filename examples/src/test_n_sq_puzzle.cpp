#include <iostream>

#include <n_sq_puzzle.hpp>

using namespace std;
using cds::n_sq_puzzle;

template <size_t N>
void test_n_sq_puzzle()
{
	using puzzle_t = n_sq_puzzle<N>;
	using Move = typename puzzle_t::MoveType;

	puzzle_t p;
	cout << "Puzzle p " << (p.is_solved() ? "(solved)" : "(not solved)") << endl;
	cout << p << endl;

	//p.shuffle(0xdeadbeee);
	p.shuffle();

	// Shuffle again
	p.shuffle();

	cout << "p after shuffle " << (p.is_solved() ? "(solved)" : "(not solved)") << endl;
	cout << p << endl;

	cout << "Move right: " << endl;
	p.move(Move::RIGHT);
	cout << p << endl;

	cout << "Move left: " << endl;
	p.move(Move::LEFT);
	cout << p << endl;

	cout << "Move up: " << endl;
	p.move(Move::UP);
	cout << p << endl;

	cout << "Move down: " << endl;
	p.move(Move::DOWN);
	cout << p << endl;


}

int main(int argc, char** argv)
{
	size_t n = 3;
	if (argc >= 2)
		n = std::atoi(argv[1]);

	if (n < 2 || n > 12)
	{
		std::cout << "n must be >= 2 and <= 12" << endl;
		return 0;
	}

	switch (n)
	{
		case 2:
			test_n_sq_puzzle<2>();
			break;
		case 3:
			test_n_sq_puzzle<3>();
			break;
		case 4:
			test_n_sq_puzzle<4>();
			break;
		case 5:
			test_n_sq_puzzle<5>();
			break;
		case 6:
			test_n_sq_puzzle<6>();
			break;
		case 7:
			test_n_sq_puzzle<7>();
			break;
		case 8:
			test_n_sq_puzzle<8>();
			break;
		case 9:
			test_n_sq_puzzle<9>();
			break;
		case 10:
			test_n_sq_puzzle<10>();
			break;
		case 11:
			test_n_sq_puzzle<11>();
			break;
		case 12:
			test_n_sq_puzzle<12>();
			break;
		default:
			std::cout << "Unknown puzzle dimension " << n << endl;
	}

	return 0;
}
