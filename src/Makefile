platform=$(shell uname -s 2>/dev/null)
platform_lcase=$(shell uname -s 2>/dev/null | tr [:upper:] [:lower:])
arch=$(shell uname -m)
# option: target_arch - if set to 32, force compilation for 32 bit architecture
ifeq ($(target_arch), 32)
	arch=i686
endif
IDIR=.
CC=gcc 
ifeq ($(platform), Linux)
	CFLAGS=-Wall -Wextra -std=c99 -pedantic -Wmissing-prototypes \
	-Wstrict-prototypes -Wold-style-definition \
    -D_POSIX_C_SOURCE=200112L -D_BSD_SOURCE
	LIBS=-lglfw -lGL -lGLU -lm -lpthread -lglut
else ifeq ($(platform), Darwin)
	INC=-I/usr/local/include
	CFLAGS=-Wall -Wextra -std=c99 -pedantic -Wmissing-prototypes \
	-Wstrict-prototypes -Wold-style-definition -O3 -Wno-deprecated-declarations #-g
	LIBS=-L/usr/local/lib -lglfw -framework OpenGL -framework GLUT \
	-framework Foundation -framework Cocoa -framework IOKit
endif

all: seewaves

osx_profile:
	iprofiler -timeprofiler -allocations -leaks -activitymonitor -systemtrace -d ${HOME}/tmp ./seewaves

ODIR=obj
LDIR =../lib


_DEPS = ptp.h cfg.h ArcBall.h Quaternion.h heartbeat.h seewaves.h Matrix.h Vector.h data_thread.h util.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = seewaves.o cfg.o Matrix.o Vector.o ArcBall.o Quaternion.o util.o heartbeat.o data_thread.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

seewaves: $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 

