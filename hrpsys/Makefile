include $(shell rospack find mk)/cmake.mk

wipe: clean
	make -f Makefile.hrpsys-base wipe
	rm -fr build
	touch wiped

clean:
	make -f Makefile.hrpsys-base clean
	-rm installed patched

