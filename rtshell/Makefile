include $(shell rospack find mk)/cmake.mk

INSTALL_DIR = `rospack find rtshell`
SOURCE_DIR  = build/rtshell-3.0.0
TARBALL     = build/rtshell-3.0.0.zip
TARBALL_URL = https://github.com/gbiggs/rtshell/archive/3.0.0.zip
MD5SUM_FILE = rtshell-3.0.0.zip.md5sum

UNPACK_CMD  = unzip

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	(cd $(SOURCE_DIR) && yes | python setup.py build && yes | python setup.py install --prefix $(INSTALL_DIR) --record installed_files.txt)
	touch installed

clean:
	(cd $(SOURCE_DIR) && cat installed_files.txt | xargs rm -rvf )
	rm -fr bin lib share installed

wipe:
	rm -fr build

