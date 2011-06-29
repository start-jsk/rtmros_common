include $(shell rospack find mk)/cmake.mk

wipe: clean
	make -f Makefile.OpenINVENT wipe
	rm -fr build share
	touch wiped

clean:
	-cd build && make clean
	make -f Makefile.OpenINVENT clean


