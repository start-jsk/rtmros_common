all: installed

FILENAME = tvmet-1.7.2.tar.bz2
TARBALL = build/$(FILENAME)
TARBALL_URL = http://downloads.sourceforge.net/tvmet/$(FILENAME)
SOURCE_DIR = build/tvmet-1.7.2
UNPACK_CMD = tar xjf
MD5SUM_FILE = $(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk


installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --prefix=$(CURDIR)
	cd $(SOURCE_DIR) && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

clean:
	if [ -e $(SOURCE_DIR) ] ; then cd $(SOURCE_DIR) && make clean; fi

wipe: clean
	rm -rf build include share bin

wiped: Makefile
	make wipe
	touch wiped

.PHONY : clean download
