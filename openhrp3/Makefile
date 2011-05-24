include $(shell rospack find mk)/cmake.mk

clean: extra-clean

wipe: clean
	rm -rf build
	touch wiped

extra-clean:
	make -f Makefile.openhrp-aist-grx clean
	make -f Makefile.tvmet clean
