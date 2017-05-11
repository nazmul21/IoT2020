# Compiler to use
CC = gcc

# Compiler Flags
# -g, used to add debug info to the executable file
# -Wall, used to display most posible warnings
CFLAGS = -g -Wall

# External link flags
# -lmraa, Intel libmraa for low speed peripherals
# -sqlite3, SQLite3 api
LFLAGS = -lmraa -lsqlite3

# Application file name
# NOTE: Output (executable) file name will aslo be the same
TARGET = app

all: $(TARGET)

$(TARGET): $(TARGET).c
	$(CC) $(CFLAGS) $(TARGET).c -o $(TARGET) $(LFLAGS)
