PREFIX ?= /usr/local
bindir = $(PREFIX)/bin
mandir = $(PREFIX)/share/man/man1

CXXFLAGS ?= -Wall -g -O3

VERSION=0.1.1

###############################################################################

ifeq ($(shell pkg-config --exists jack || echo no), no)
  $(error "http://jackaudio.org/ is required - install libjack-dev or libjack-jackd2-dev")
endif
ifeq ($(shell pkg-config --exists sndfile || echo no), no)
  $(error "http://www.mega-nerd.com/libsndfile/ is required - install libsndfile1-dev")
endif
ifeq ($(shell pkg-config --exists fftw3f || echo no), no)
  $(error "http://fftw.org/ is required - install libfftw3-dev")
endif

CXXFLAGS+=`pkg-config --cflags jack sndfile fftw3f` -pthread
LOADLIBES=`pkg-config --libs jack sndfile fftw3f` -lm

CPPFLAGS+=-Izita/
CPPFLAGS+=-DVERSION=\"$(VERSION)\"

all: jack-ir

man: jack-ir.1

jack-ir: jack-ir.cc zita/zita-convolver.cc

jack-ir.1: jack-ir
	help2man -N -n 'JACK Impulse Response Recorder' -o jack-ir.1 ./jack-ir

clean:
	rm -f jack-ir

install: install-bin install-man

uninstall: uninstall-bin uninstall-man

install-bin: jack-ir
	install -d $(DESTDIR)$(bindir)
	install -m755 jack-ir $(DESTDIR)$(bindir)

uninstall-bin:
	rm -f $(DESTDIR)$(bindir)/jack-ir
	-rmdir $(DESTDIR)$(bindir)

install-man:
	install -d $(DESTDIR)$(mandir)
	install -m644 jack-ir.1 $(DESTDIR)$(mandir)

uninstall-man:
	rm -f $(DESTDIR)$(mandir)/jack-ir.1
	-rmdir $(DESTDIR)$(mandir)

.PHONY: all clean install uninstall man install-man install-bin uninstall-man uninstall-bin
