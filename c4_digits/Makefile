SUBDIRS := $(shell find . -mindepth 1 -maxdepth 1 -type d ! -iname "imgs")

all:
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir; \
	done

clean:
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean; \
	done

.PHONY: all clean
