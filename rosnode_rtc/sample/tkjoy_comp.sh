#!/bin/bash -i

export RTCTREE_NAMESERVERS=localhost
export PATH=$PATH:`rospack find openrtm`/bin
export PYTHONPATH=$PYTHONPATH:`rospack find openrtm`/src/openrtm
export PYTHONPATH=$PYTHONPATH:`rospack find openrtm`/lib/python2.7/site-packages/OpenRTM_aist/RTM_IDL/

roscd openrtm/build/OpenRTM-aist-Python-1.0.1/OpenRTM_aist/examples/TkJoyStick
python TkJoyStickComp.py
