include $(shell rospack find mk)/cmake.mk

wipe: clean
	# make -f Makefile.hrpsys-base wipe
	rm -fr build share
	touch wiped

clean:
	make -f Makefile.hrpsys-base clean
	-rm -fr installed patched include bin lib idl idl_gen

