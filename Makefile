CXX?=g++

all: main.cpp
	$(CXX) -Wall -Wextra -pipe -O3 -ggdb -pedantic -std=c++2a main.cpp -o main.out `pkg-config --libs sdl2` -lm `pkg-config --cflags eigen3`
debug: main.cpp
	$(CXX) -Wall -Wextra -pipe -O0 -ggdb -pedantic -std=c++2a main.cpp -o main.out `pkg-config --libs sdl2` -lm `pkg-config --cflags eigen3`


clean:
	-rm main.out
