include $(shell rospack find mk)/cmake.mk

wipe: clean
	make -f Makefile.openhrp-aist-grx wipe
	rm -fr build
	touch wiped

clean:
	make -f Makefile.openhrp-aist-grx clean


