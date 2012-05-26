CC=g++
CFLAGS=-DTESTING -g -Wall -Werror
EXE=test
OBJS=FuriousBoard.o serial232.o

$(EXE): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

FuriousBoard.o: FuriousBoard.cpp
	$(CC) $(CFLAGS) -c $<

serial232.o: serial232.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(EXE) $(OBJS)
