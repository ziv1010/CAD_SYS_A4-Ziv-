# Makefile for ENGDRAW_COPY_2 project

# Compiler and linker
CXX = g++
CC = gcc

# Directories
SRC_DIR = src
INCLUDE_DIR = include
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/.o
OUTPUT_DIR = $(BUILD_DIR)/output
DEPENDENCIES_DIR = dependencies

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
OBJS += $(OBJ_DIR)/glad.o

# Executable
TARGET = $(OUTPUT_DIR)/app

# Include paths
INCLUDE_PATHS = -I$(INCLUDE_DIR) \
                -I$(DEPENDENCIES_DIR)/include \
                -I/opt/homebrew/opt/glfw/include \
                -I/opt/homebrew/opt/glew/include \
                -I/opt/homebrew/opt/glm/include

# Library paths
LIBRARY_PATHS = -L$(DEPENDENCIES_DIR)/library \
                -L/opt/homebrew/opt/glfw/lib \
                -L/opt/homebrew/opt/glew/lib \
                -L/opt/homebrew/opt/glm/lib

# Libraries to link
LIBS = -lglfw -lGLEW -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo

# Compiler flags for C++
CXXFLAGS = -std=c++17 -Wall -g -DGLM_ENABLE_EXPERIMENTAL -Wno-deprecated-declarations $(INCLUDE_PATHS)

# Compiler flags for C
CFLAGS = -Wall -g -Wno-deprecated-declarations $(INCLUDE_PATHS)

# Build rules
all: $(TARGET)

# Create build directories if they don't exist
$(OBJ_DIR) $(OUTPUT_DIR):
	mkdir -p $@

# Compile .cpp files to .o files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile glad.c separately using C compiler and CFLAGS
$(OBJ_DIR)/glad.o: $(SRC_DIR)/glad.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link the object files to create the executable
$(TARGET): $(OBJS) | $(OUTPUT_DIR)
	$(CXX) $(CXXFLAGS) $(LIBRARY_PATHS) $^ -o $@ $(LIBS)

# Clean up build artifacts
clean:
	rm -rf $(OBJ_DIR)/*.o $(OUTPUT_DIR)/app

.PHONY: all clean
