CC = g++
CFLAGS = -g -O3 -std=c++17 -lGL -lGLU -lglfw -lassimp
M = 0
DM = 0
SM = 0

run: simulation
	./simulation $(M) $(DM) $(SM)

simulation: main.cpp src/*.cpp include/*.hpp
	$(CC) -o simulation main.cpp src/*.cpp $(CFLAGS)

clean:
	rm -f simulation
