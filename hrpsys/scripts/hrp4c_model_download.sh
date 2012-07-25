#!/bin/bash

HRP4C_DOWNLOAD_FILE=$HOME/Downloads/HRP-4C.zip
HRP4C_MODEL_DIR=`rospack find hrpsys`/share/hrpsys/samples/
HRP4C_MODEL_PATH=${HRP4C_MODEL_DIR}/HRP4C/HRP4Cmain.wrl

if [ ! -f ${HRP4C_MODEL_PATH} ]; then
    if [ -f ${HRP4C_DOWNLOAD_FILE} ]; then
	unzip ${HRP4C_DOWNLOAD_FILE} -d ${HRP4C_MODEL_DIR}
	rm -fr ${HRP4C_MODEL_DIR}/HRP4C
	mv ${HRP4C_MODEL_DIR}/HRP-4C ${HRP4C_MODEL_DIR}/HRP4C
    else
	echo "======================================================"
	echo "======================================================"
	echo "======================================================"
	echo "Download HRP4 model from http://unit.aist.go.jp/is/humanoid/hrp-4c/hrp-4c.html and extract to ${HRP4C_MODEL_PATH}"
	echo "======================================================"
	echo "======================================================"
	echo "======================================================"
	firefox http://unit.aist.go.jp/is/humanoid/hrp-4c/hrp-4c.html
	firefox http://unit.aist.go.jp/is/humanoid/hrp-4c/agreement.html
	exit -1
    fi
fi
if [ -f ${HRP4C_MODEL_PATH} ]; then
    (cd ${HRP4C_MODEL_DIR}; patch -N HRP4C/HRP4Cmain.wrl < `rospack find hrpsys`/patch/HRP4Cmain.view.patch)
    (cd ${HRP4C_MODEL_DIR}/HRP4C/cover; sed -i 's/^\(.*ccw.*\)$/#\1/' L_*.wrl  CHEST_*.wrl WAIST.wrl NECK_*.wrl)
    echo 0
fi

