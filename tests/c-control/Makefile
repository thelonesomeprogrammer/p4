# Variables
CC = gcc
CFLAGS = -Wall
SRC = main.c pid.c lead.c
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

