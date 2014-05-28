# You know, there are pre-compile DEBs of this...

all: build

build:
	$(MAKE) -C qtsixa
	$(MAKE) -C utils
	$(MAKE) -C sixad

clean:
	$(MAKE) clean -C qtsixa
	$(MAKE) clean -C utils
	$(MAKE) clean -C sixad

install:
	$(MAKE) install -C qtsixa
	$(MAKE) install -C utils
	$(MAKE) install -C sixad

uninstall:
	$(MAKE) uninstall -C sixad
	$(MAKE) uninstall -C utils
	$(MAKE) uninstall -C qtsixa

