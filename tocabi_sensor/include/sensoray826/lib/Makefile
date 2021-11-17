CC=gcc
CFLAGS+=-D_LINUX

LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
s826demo: s826_example.o lib826_64.a
	$(CC) $(CFLAGS) s826_example.o -o s826demo -lm -L. -l826_64
else
s826demo: s826_example.o lib826.a
	$(CC) $(CFLAGS) s826_example.o -o s826demo -lm -L. -l826
endif

all: s826demo




s826_example.o: s826_example.c
	$(CC) $(CFLAGS)  -Wall -Wextra -DOSTYPE_LINUX -c  s826_example.c -o s826_example.o


clean:
	rm -rf s826_example.o s826demo
