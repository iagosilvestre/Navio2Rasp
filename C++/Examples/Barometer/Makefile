CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) $(INCLUDES) -Wall -DWITHGPERFTOOLS -lprofiler -g Barometer.cpp -L$(NAVIO) -lnavio -o Barometer

clean:
	rm -f Barometer
