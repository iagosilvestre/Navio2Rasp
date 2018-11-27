CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) $(INCLUDES) -Wall -std=c99 -pg test_gprof.c -L$(NAVIO) -lnavio -o test_gprof
clean:
	rm -f test_gprof
