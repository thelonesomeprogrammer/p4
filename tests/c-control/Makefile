# Variables
CC = gcc
CFLAGS = -Wall -lm
SRC = main.c chat_sim.c
OBJ = $(SRC:.c=.o)
TARGET = myapp

# Default target
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

