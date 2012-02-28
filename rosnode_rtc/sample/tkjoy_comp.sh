#!/bin/bash -i

roscd openrtm/build/OpenRTM-aist-Python-1.0.1/OpenRTM_aist/examples/TkJoyStick
PYTHONPATH=$PYTHONPATH:`rospack export --lang=python --attrib=path openrtm | cut -d\  -f1` python TkJoyStickComp.py
