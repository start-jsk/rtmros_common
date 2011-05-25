include $(shell rospack find mk)/cmake.mk

clean: extra-clean

wipe: clean
	make -f Makefile.openhrp-aist-grx wipe
	touch wiped

extra-clean:
	make -f Makefile.openhrp-aist-grx clean
