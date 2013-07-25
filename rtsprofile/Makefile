include $(shell rospack find mk)/cmake.mk

INSTALL_DIR = `rospack find rtspfofile`
SOURCE_DIR  = build/rtsprofile-2.0.0
TARBALL     = build/rtsprofile-2.0.0.zip
TARBALL_URL = https://github.com/gbiggs/rtsprofile/archive/2.0.0.zip
MD5SUM_FILE = rtsprofile-2.0.0.zip.md5sum

UNPACK_CMD  = unzip

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	(cd $(SOURCE_DIR) && python setup.py build && python setup.py install --prefix ../../src)
	touch installed

clean:
	rm -fr build src

