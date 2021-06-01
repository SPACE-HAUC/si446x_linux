CC=$(CROSS_COMPILE)gcc
EDCFLAGS := -Wall -O2 -I include/ -I drivers/ -pthread -std=gnu11 -Wno-unused-function -Wno-unused-variable $(CFLAGS)
EDLDFLAGS := -lpthread -lm $(LDFLAGS)

COBJS = drivers/gpiodev/gpiodev.o \
		drivers/spibus/spibus.o \
		src/Si446x.o
	
LIBTARGET = libsi446x.a

all: $(LIBTARGET)

$(LIBTARGET): $(COBJS)
	ar -crus $(LIBTARGET) $(COBJS)

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

.PHONY: clean doc spotless

clean:
	rm -vf $(LIBTARGET)
	rm -vf $(COBJS)

spotless: clean
	rm -rf doc

doc:
	doxygen .doxyconfig