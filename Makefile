CXX=g++
CFLAGS=-Wall -O3 -std=c++14
CFLAGS_DEBUG=-Wall -O0 -g -std=c++14
OUTDIR=Release
OUTDIR_DEBUG=Debug

STUFF=test_a_star_search test_n_sq_puzzle solve_n_sq_puzzle test_cycle_decomposition

all: $(STUFF)

$(STUFF): %: %.cpp
	$(CXX) $(CFLAGS) -o $(OUTDIR)/$@ $<

debug: OUTDIR=$(OUTDIR_DEBUG)
debug: CFLAGS=$(CFLAGS_DEBUG)
debug: $(STUFF)

clean:
	rm -f $(OUTDIR)/*

debug_clean: OUTDIR=$(OUTDIR_DEBUG)
debug_clean: clean
