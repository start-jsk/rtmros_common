include $(shell rospack find mk)/cmake.mk

##
## hrpsys-base (http://hrpsys-base.googlecode.com/svn/trunk/)
## revision newer than 83 is required
##

wipe: clean
	# make -f Makefile.hrpsys-base wipe
	rm -fr build share
	touch wiped

clean: Makefile.hrpsys-base
	make -f Makefile.hrpsys-base clean
	-rm -fr installed patched include bin lib idl idl_gen msg msg_gen srv srv_gen src_gen

