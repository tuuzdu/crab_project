# You know, there are pre-compile DEBs of this...

CC ?= gcc
CFLAGS ?= -O2 -Wall
LDFLAGS += -Wl,-Bsymbolic-functions

WANT_JACK = true

all: tools

tools:
	mkdir -p bins
	$(CC) $(CFLAGS) $(LDFLAGS) hidraw-dump.c -o bins/hidraw-dump
	$(CC) $(CFLAGS) $(LDFLAGS) sixpair.c -o bins/sixpair `pkg-config --cflags --libs libusb`
	$(CC) $(CFLAGS) $(LDFLAGS) sixpair_kbd.c -o bins/sixpair-kbd `pkg-config --cflags --libs libusb`
ifeq ($(WANT_JACK),true)
	$(CC) $(CFLAGS) $(LDFLAGS) sixad-jack.c -o bins/sixad-jack `pkg-config --cflags --libs jack` -lm
endif

clean:
	rm -f *~ bins/*

install:
	install -d $(DESTDIR)/usr/bin/
	install -d $(DESTDIR)/usr/sbin/
	install -m 755 bins/sixpair $(DESTDIR)/usr/sbin/
	install -m 755 bins/sixpair-kbd $(DESTDIR)/usr/sbin/
	install -m 755 bins/hidraw-dump $(DESTDIR)/usr/sbin/
ifeq ($(WANT_JACK),true)
	install -m 755 bins/sixad-jack $(DESTDIR)/usr/bin/
endif

uninstall:
	rm -f $(DESTDIR)/usr/bin/sixad-jack
	rm -f $(DESTDIR)/usr/sbin/hidraw-dump
	rm -f $(DESTDIR)/usr/sbin/sixpair
	rm -f $(DESTDIR)/usr/sbin/sixpair-kbd
