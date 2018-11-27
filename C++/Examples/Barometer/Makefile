CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) $(INCLUDES) -Wall -pg -fprofile-arcs -ftest-coverage Barometer.cpp -L$(NAVIO) -lnavio -o Barometer

clean:
	rm -f Barometer
