CC=gcc

all: dice

dice: dice.c
	$(CC) -Wall -ggdb -o dice dice.c

clean:
	rm -rf dice

