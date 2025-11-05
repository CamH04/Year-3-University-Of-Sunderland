# Compiler and flags
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -I./include
LDFLAGS :=

# Directories
SRC_DIR := src
INC_DIR := include
OBJ_DIR := obj
BIN_DIR := bin

# Target executable
TARGET := $(BIN_DIR)/program

# Source files
MAIN_SRC := main.cpp
SRC_FILES := $(wildcard $(SRC_DIR)/*.cpp)

# Object files
MAIN_OBJ := $(OBJ_DIR)/main.o
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES))

# All object files
ALL_OBJS := $(MAIN_OBJ) $(OBJ_FILES)

all: $(TARGET)

$(TARGET): $(ALL_OBJS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(ALL_OBJS) -o $(TARGET) $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

$(MAIN_OBJ): $(MAIN_SRC) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)
	@echo "Clean complete"

run: $(TARGET)
	./$(TARGET)

rebuild: clean all

debug: CXXFLAGS += -g -O0 -DDEBUG
debug: clean all

release: CXXFLAGS += -O3 -DNDEBUG
release: clean all

print-%:
	@echo $* = $($*)

.PHONY: all clean run rebuild debug release print-%
