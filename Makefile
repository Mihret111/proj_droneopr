CC = gcc
CFLAGS = -Wall -Wextra -g -Iheaders -I.
LDFLAGS = -lncurses -lm
TARGET = arp1
BUILD_DIR = build

# Source files
SRCS = src/main.c src/server.c src/dynamics.c src/keyboard.c src/obstacles.c src/targets.c src/watchdog.c src/params.c src/util.c

# Object files
OBJS = $(patsubst src/%.c, $(BUILD_DIR)/%.o, $(SRCS))

# Default target
.PHONY: all
all: $(TARGET)

# Link the executable
$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

# Compile source files into object files
$(BUILD_DIR)/%.o: src/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build artifacts
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# Run the application
.PHONY: run
run: $(TARGET)
	./$(TARGET)

# Help target
.PHONY: help
help:
	@echo "Makefile for $(TARGET)"
	@echo "Usage:"
	@echo "  make        Build the executable"
	@echo "  make clean  Remove object files and executable"
	@echo "  make run    Build and run the program"
	@echo "  make help   Show this help message"
