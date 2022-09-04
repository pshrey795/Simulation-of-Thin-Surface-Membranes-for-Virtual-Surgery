CC = g++
CFLAGS = -lGL -lGLU -lglfw
MODE = 0

run: simulation
	./simulation $(MODE)

simulation: main.cpp src/*.cpp
	$(CC) -o simulation main.cpp src/*.cpp $(CFLAGS)

clean:
	rm -f simulation
