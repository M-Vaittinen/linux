CC=gcc
LDFLAGS=-L/usr/lib64/mysql -lmysqlclient
#/usr/lib64/mysql/libmysqlclient.so

all: diceadmin-public

diceadmin: diceadmin-public.c
	$(CC) -Wall -ggdb -o diceadmin-public diceadmin-public.c $(LDFLAGS)

clean:
	rm -rf diceadmin-public

