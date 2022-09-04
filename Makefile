CC = g++
CFLAGS = -lGL -lGLU -lglfw
MODE = 0

run: simulation
	./simulation $(MODE)

simulation: main.cpp src/mesh.cpp src/half_edge.cpp src/path.cpp src/curve.cpp
	$(CC) -o simulation main.cpp $(CFLAGS)

clean:
	rm -f simulation
