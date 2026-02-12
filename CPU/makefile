CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -lX11 -lGL -lpthread -lpng -lstdc++fs
TARGET = nes

SRC = main.cpp Bus.cpp 6502.cpp
OBJ = $(SRC:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

run: all
	./$(TARGET)
