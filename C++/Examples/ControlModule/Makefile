CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) -std=c++11 -Wall -DWITHGPERFTOOLS -lprofiler -g $(INCLUDES) -L$(NAVIO) controlM.cpp -L$(NAVIO) -lnavio -lpthread -o controlM -static

clean:
	rm -f controlM
