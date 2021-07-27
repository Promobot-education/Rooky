APP1 = move_joints
APP2 = read_servos

TARGETS = $(APP1) $(APP2)

SOURCE_COMMON = $(wildcard ./src/*.cpp)
SOURCE_APP1 = $(SOURCE_COMMON) $(wildcard ./examples/move_joints.cpp)
SOURCE_APP2 = $(SOURCE_COMMON) $(wildcard ./examples/read_servos.cpp)

OBJ_APP1 = $(patsubst %.c, %.o, $(SOURCE_APP1))
OBJ_APP2 = $(patsubst %.c, %.o, $(SOURCE_APP2))

INCLUDE_COMMON = -I./include -I/usr/local/include/modbus -L/usr/local/lib -lmodbus
FLAGS = -Wall -c
CXX = g++
CXXFLAGS = -std=c++11 

all: $(TARGETS)
$(APP1): $(OBJ_APP1)	
	@mkdir -p build/
	$(CXX) $(CXXFLAGS) $(OBJ_APP1) $(INCLUDE_COMMON) -o build/$(APP1)

$(APP2): $(OBJ_APP2)	
	@mkdir -p build/
	$(CXX) $(CXXFLAGS) $(OBJ_APP2) $(INCLUDE_COMMON) -o build/$(APP2)

%.o: %.c
	$(CXX) $(CXXFLAGS) $(INCLUDE_COMMON) $(FLAGS) $< -o $@

.PHONY: clean
clean:
	rm -f *.o
	rm -rf $(OBJ_APP1) $(OBJ_APP2) build/ 