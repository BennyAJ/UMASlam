CXX = g++
FLAGS = -Wall -Wno-unused-function -Werror -pthread -std=c++11
OPTIMIZATION_FLAGS = -O3
PFLAGS = -pg -DPROFILE -O3
SFML_FLAGS = -lsfml-graphics -lsfml-window -lsfml-system
LCM_FLAGS = -llcm
BIN_PATH = ./bin
OBJ_PATH = ./obj
SRC_PATH = ./src
TEST_PATH = ./unit_tests

all: Localizer Main IMUTransformer PerfectStateGenerator

optimized: FLAGS += $(OPTIMIZATION_FLAGS)
optimized: all

debug: FLAGS += -g3
debug: all

profile: FLAGS += $(PFLAGS)
profile: all

obj/%.o: $(SRC_PATH)/%.cpp
	$(CXX) $(FLAGS) -c  $^ -o $@

obj/%.o: $(TEST_PATH)/%.cpp
	$(CXX) $(FLAGS) -c  $^ -o $@

Localizer: $(OBJ_PATH)/Localizer.o

SLAM: $(OBJ_PATH)/SLAM.o

Main: $(OBJ_PATH)/SLAM.o $(OBJ_PATH)/Localizer.o $(OBJ_PATH)/MapDrawer.o $(OBJ_PATH)/Main.o $(OBJ_PATH)/CoordTransformer.o 	
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/SLAM $(SFML_FLAGS) $(LCM_FLAGS)

IMUTransformer: $(OBJ_PATH)/IMUTransformer.o $(OBJ_PATH)/CoordTransformer.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/IMUTransformer $(LCM_FLAGS)

PerfectStateGenerator: $(OBJ_PATH)/PerfectStateGenerator.o $(OBJ_PATH)/CoordTransformer.o
	$(CXX) $(FLAGS) $^ -o $(BIN_PATH)/PerfectStateGenerator $(LCM_FLAGS)

report:
	$(MAKE) -C report 

clean:
	- rm bin/*
	- rm obj/*
