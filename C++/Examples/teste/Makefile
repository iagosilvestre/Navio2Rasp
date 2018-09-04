CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) $(INCLUDES) teste.cpp -L$(NAVIO) -lnavio -o teste

clean:
	rm -f teste
