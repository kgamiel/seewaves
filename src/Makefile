platform=$(shell uname -s 2>/dev/null)
platform_lcase=$(shell uname -s 2>/dev/null | tr [:upper:] [:lower:])
arch=$(shell uname -m)
# option: target_arch - if set to 32, force compilation for 32 bit architecture
ifeq ($(target_arch), 32)
	arch=i686
endif
IDIR = .
CC=gcc
ifeq ($(platform), Linux)
	CFLAGS=-Wall -Wextra -std=c89 -pedantic -Wmissing-prototypes \
	-Wstrict-prototypes -Wold-style-definition -I$(IDIR) -O3 \
    -D_POSIX_C_SOURCE=200112L -D_BSD_SOURCE
	LIBS=-lglfw -lGL -lGLU -lm -lpthread -lglut
else ifeq ($(platform), Darwin)
	CFLAGS=-Wall -Wextra -std=c89 -pedantic -Wmissing-prototypes \
	-Wstrict-prototypes -Wold-style-definition -I$(IDIR) -O3
	LIBS=-lglfw -framework OpenGL -framework GLUT
endif



ODIR=obj
LDIR =../lib


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

