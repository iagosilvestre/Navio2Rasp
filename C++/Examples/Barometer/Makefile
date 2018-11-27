CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) $(INCLUDES) -Wall -std=c99 -pg Barometer.cpp -L$(NAVIO) -lnavio -o Barometer

clean:
	rm -f Barometer
