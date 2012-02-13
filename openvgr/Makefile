Ball: installed all
include $(shell rospack find mk)/cmake.mk

INSTALL_DIR = $(CURDIR)
FILENAME    = OpenVGR-0.9.0.tgz
TARBALL     = build/$(FILENAME)
TARBALL_URL = http://openvgr.googlecode.com/files/$(FILENAME)
SOURCE_DIR  = build/OpenVGR-0.9.0
TARBALL_PATCH = OpenVGR-0.9.0.src.patch OpenVGR-0.9.0.example.patch
UNPACK_CMD  = tar xfz
#MD5SUM_FILE = $(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

OPENCV20_DIR = $(shell rospack find openvgr)/build/OpenCV-2.0.0

installed: $(SOURCE_DIR)/unpacked opencv20
	cd $(SOURCE_DIR) && PATH=`rospack find openrtm`/bin:$$PATH PKG_CONFIG_PATH=$(OPENCV20_DIR)/installed/lib/pkgconfig/:$$PKG_CONFIG_PATH make && cd build && make && cp -R ../example/dataset/image ./ && cp -R ../example/dataset/calib ./ && cp -R ../example/model ./ && cp -R ../example/dataset/param ./
	sed -i 's@^source /usr/local/share/rtshell/@#source /usr/local/share/rtshell/shell_support@' $(SOURCE_DIR)/example/script/*.sh $(SOURCE_DIR)/example/script/*/*.sh
	sed -i 's@#!/bin/bash@#!/bin/bash\nexport LD_LIBRARY_PATH=`rospack find openrtm`/lib:$(OPENCV20_DIR)/installed/lib:$$LD_LIBRARY_PATH\nexport PATH=$$PATH:`rospack find openrtm`/bin\nexport PYTHONPATH=$$PYTHONPATH:`rospack find openrtm`/src/openrtm\nsource `rospack find openrtm`/scripts/rtshell-setup.sh@' $(SOURCE_DIR)/example/script/*.sh $(SOURCE_DIR)/example/script/substill/*.sh

stilltest:
	cd $(SOURCE_DIR)/example/script && LD_LIBRARY_PATH=$(OPENCV20_DIR)/installed/lib/:$(LD_LIBRARY_PATH) ./stilltest.sh

opencv20:
	if [ ! -e build/OpenCV-2.0.0 ]; then \
		if [ ! -e build/OpenCV-2.0.0.tar.bz2 ]; then \
			wget http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.0/OpenCV-2.0.0.tar.bz2 -P build ;\
		fi ;\
		cd build ;\
		tar -xjf OpenCV-2.0.0.tar.bz2 ;\
		cd OpenCV-2.0.0 ;\
		patch -p1 < $(CURDIR)/cxcore.hpp.patch ;\
		cmake -DCMAKE_INSTALL_PREFIX:PATH=$(OPENCV20_DIR)/installed -DWITH_FFMPEG=OFF -DWITH_1394=OFF -DWITH_GSTREAMER=OFF -DWITH_V4L=OFF ;\
		make -j4 ;\
		make install ;\
	fi

clean:
	-cd $(SOURCE_DIR) && make clean
	rm -fr installed

wipe: clean
	rm -fr build bin include lib share
