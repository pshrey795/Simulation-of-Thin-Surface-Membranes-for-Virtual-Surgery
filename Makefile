CC = g++ -O3
CFLAGS = -g -O3 -std=c++17
LDFLAGS = -lGL -lGLU -lglfw -lassimp
EXEC = simulation
SOURCES = $(wildcard src/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)
M = 0
DM = 0
SM = 0

$(EXEC): $(OBJECTS) main.cpp include/*.hpp
	$(CC) $(OBJECTS) main.cpp -o $(EXEC) $(LDFLAGS)

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

run: $(OBJECTS) main.cpp include/*.hpp
	$(CC) $(OBJECTS) main.cpp -o $(EXEC) $(LDFLAGS)
	./$(EXEC) $(M) $(DM) $(SM)

simulate: $(EXEC)
	./$(EXEC) $(M) $(DM) $(SM)

clean:
	rm -f $(EXEC) $(OBJECTS)
