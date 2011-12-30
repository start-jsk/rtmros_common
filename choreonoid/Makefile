Ball: installed all
include $(shell rospack find mk)/cmake.mk

INSTALL_DIR = $(CURDIR)
FILENAME    = choreonoid-1.1.0.zip
TARBALL     = build/$(FILENAME)
TARBALL_URL = http://choreonoid.org/_downloads/$(FILENAME)
SOURCE_DIR  = build/choreonoid-1.1.0
TARBALL_PATCH =
UNPACK_CMD  = unzip
MD5SUM_FILE = $(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && cmake . -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) -DENABLE_INSTALL_RPATH=ON && make && make install

clean:
	-cd $(SOURCE_DIR) && make clean
	rm -fr installed

wipe: clean
	rm -fr build bin include lib share
