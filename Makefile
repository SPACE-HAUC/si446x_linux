CC=$(CROSS_COMPILE)gcc
EDCFLAGS := -Wall -O2 -I include/ -I drivers/ -pthread -std=gnu11 -Wno-unused-function -Wno-unused-variable $(CFLAGS)
EDLDFLAGS := -lpthread -lm $(LDFLAGS)

COBJS = drivers/gpiodev/gpiodev.o \
		drivers/spibus/spibus.o \
		src/si446x.o
	
LIBTARGET = libsi446x.a

ifeq ($(INSTALLDIR),)
INSTALLDIR := /usr/local
endif

all: lib basic_test

lib: $(LIBTARGET)

basic_test: basic_test.o $(LIBTARGET)
	@$(CC) -o $@.out $< $(LIBTARGET) $(EDLDFLAGS)
	@echo "$@.out built"

$(LIBTARGET): $(COBJS)
	@ar -crus $(LIBTARGET) $(COBJS)
	@echo "$(LIBTARGET) built"

install: $(LIBTARGET)
	cp include/si446x.h $(INSTALLDIR)/include/
	cp $(LIBTARGET) $(INSTALLDIR)/lib/
	@echo "Installed to $(INSTALLDIR)/include and $(INSTALLDIR)/lib"
	@echo "Ensure includepath and link path are set correctly for other programs"
	@echo "Change of radio configuration will require recompilation and reinstallation of the library"

# remove library only
uninstall:
	rm -vf $(INSTALLDIR)/lib/$(LIBTARGET) 

%.o: %.c
	@$(CC) $(EDCFLAGS) -o $@ -c $<

.PHONY: clean doc spotless

clean:
	rm -vf $(LIBTARGET)
	rm -vf $(COBJS)
	rm -vf *.out
	rm -vf *.o

spotless: clean
	rm -rf doc

doc:
	doxygen .doxyconfig