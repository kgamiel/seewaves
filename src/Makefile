IDIR = .
CC=gcc
CFLAGS=-Wall -Wextra -std=c89 -pedantic -Wmissing-prototypes \
	-Wstrict-prototypes -Wold-style-definition -I$(IDIR) -O3

# debug flags
#CFLAGS=-g -Wall -Wextra -std=c89 -pedantic -Wmissing-prototypes \
#	-Wstrict-prototypes -Wold-style-definition -I$(IDIR)

ODIR=obj
LDIR =../lib

LIBS=-lglfw -framework OpenGL

_DEPS = ptp.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = seewaves.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

seewaves: $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 

