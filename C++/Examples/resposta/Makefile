CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) -std=c++11 $(INCLUDES) -L$(NAVIO) resposta.cpp -L$(NAVIO) -lnavio -lpthread -o resposta

clean:
	rm -f resposta
