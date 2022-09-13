CC = g++
CFLAGS =-g -O3 -std=c++17 -lGL -lGLU -lglfw -lassimp
MODE = 0

run: simulation
	./simulation $(MODE)

simulation: main.cpp src/*.cpp include/*.hpp
	$(CC) -o simulation main.cpp src/*.cpp $(CFLAGS)

clean:
	rm -f simulation
