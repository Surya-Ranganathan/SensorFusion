# Compiler settings
CXX := g++
CXXFLAGS := -std=c++11 -Wall -Iinclude

# Directories
SRC_DIR := src
OBJ_DIR := build
OUT_DIR := output
BIN := $(OBJ_DIR)/a.out

# Source and object files
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

# Default target
all: $(BIN)

# Link object files into final binary
$(BIN): $(OBJS) | $(OUT_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile .cpp to .o
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Create directories if not exist
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

# Run the program
run: $(BIN)
	./$(BIN)

# Clean
clean:
	rm -rf $(OBJ_DIR) $(OUT_DIR) $(BIN)
