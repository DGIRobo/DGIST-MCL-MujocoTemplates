#LINUX
COMMON=-O2 -I../../include -L../../lib -pthread -Wl,-rpath,'$$ORIGIN'/../../lib
LIBS = -lmujoco -lGL -lm -lnlopt -lglfw
CC = gcc

ROOT = projectile_opt

all:
	$(CC) $(COMMON) main.c $(LIBS) -o ./$(ROOT)

main.o:
	$(CC) $(COMMON) -c main.c

clean:
	rm *.o ./$(ROOT)
