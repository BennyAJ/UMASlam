CXX = g++
FLAGS = -Wall -Werror -std=c++11
SFML_FLAGS = -lsfml-graphics -lsfml-window -lsfml-system
LCM_FLAGS = -llcm
BIN_PATH = ./bin
OBJ_PATH = ./obj
SRC_PATH = ./src
TEST_PATH = ./unit_tests

all: GridTest MapperTest

debug: FLAGS += -DSLAM_DEBUG_LEVEL=$(LEVEL)
debug: all

obj/%.o: $(SRC_PATH)/%.cpp
	$(CXX) $(FLAGS) -c  $^ -o $@

obj/%.o: $(TEST_PATH)/%.cpp
	$(CXX) $(FLAGS) -c  $^ -o $@

obj/MapDrawer.o: $(SRC_PATH)/MapDrawer.cpp
	$(CXX) $(FLAGS) $(SFML_FLAGS) -c $^ -o $@

MapperTest: $(OBJ_PATH)/Mapper.o $(OBJ_PATH)/MapDrawer.o $(OBJ_PATH)/MapperTester.o $(OBJ_PATH)/GridMap.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/MapperTest $(SFML_FLAGS) $(LCM_FLAGS)

GridTest: $(OBJ_PATH)/GridMap.o $(OBJ_PATH)/GridTests.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/GridTest

test: GridTest MapperTest
	./bin/GridTest >/dev/null

clean:
	rm bin/*
	rm obj/*