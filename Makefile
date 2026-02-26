CC = g++ -O3
CFLAGS = -g -O3 -std=c++17 -I/opt/homebrew/Cellar/eigen/5.0.0/include/ -I/opt/homebrew/Cellar/glfw/3.4/include/ -I/opt/homebrew/Cellar/assimp/6.0.2/include/
LDFLAGS = -I/opt/homebrew/Cellar/eigen/5.0.0/include/ -I/opt/homebrew/Cellar/glfw/3.4/include/ -I/opt/homebrew/Cellar/assimp/6.0.2/include/ -L/opt/homebrew/lib -framework OpenGL -lglfw -lassimp
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

show: $(EXEC)
	./$(EXEC) $(M) $(DM) $(SM)

clean:
	rm -f $(EXEC) $(OBJECTS)
