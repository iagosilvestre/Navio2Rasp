CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) -std=c++11 -Wall -g $(INCLUDES) -L$(NAVIO) testeOneThread.cpp -L$(NAVIO) -lnavio -lpthread -o testeOneThread

clean:
	rm -f testeOneThread
