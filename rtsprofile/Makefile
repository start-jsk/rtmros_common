include $(shell rospack find mk)/cmake.mk

INSTALL_DIR = `rospack find rtsprofile`
SOURCE_DIR  = build/rtsprofile-2.0.0
TARBALL     = build/rtsprofile-2.0.0.zip
TARBALL_URL = https://github.com/gbiggs/rtsprofile/archive/2.0.0.zip
MD5SUM_FILE = rtsprofile-2.0.0.zip.md5sum

UNPACK_CMD  = unzip

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	(cd $(SOURCE_DIR) && python setup.py build && python setup.py install --prefix $(INSTALL_DIR) --record installed_files.txt)
	touch installed

clean:
	(cd $(SOURCE_DIR) && cat installed_files.txt | xargs rm -rvf )
	rm -fr bin lib installed

wipe:
	rm -fr build
