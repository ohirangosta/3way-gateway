CC=gcc
CFLAGS=-I.	-O
CLIB=-ll


TARGET=parser
OBJ=$(TARGET).o lexer.o

.SUFFIXES: .c .o

.PHONY: all

all:$(TARGET)

$(TARGET):$(OBJ) 
	$(CC) $(CFLAGS) -o $@ $(OBJ) $(CLIB)

.c.o:
	$(CC) $(CFLAGS) -c $<

.PHONY: clean-this


clean: clean-this

clean-this:
	/bin/rm	-f $(TARGET) $(OBJ) *~
