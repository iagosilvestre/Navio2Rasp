CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) $(INCLUDES) -pg Barometer.cpp -L$(NAVIO) -lnavio -o Barometer.o

clean:
	rm -f Barometer.o
