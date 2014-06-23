^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosnode_rtc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2014-06-23)
------------------
* (package.xml files) Update repository URLs
* Contributors: Isaac Isao Saito

1.2.0 (2014-05-14)
------------------

* bump to 1.2.0for hrpsys 315.2.0

1.0.12 (2014-05-06)
-------------------

1.0.11 (2014-04-16)
-------------------

1.0.10 (2014-04-15)
-------------------

1.0.9 (2014-04-14)
------------------

1.0.8 (2014-04-14)
------------------

1.0.7 (2014-03-20)
------------------
* (package.xml) move rostopic from test_depend to build_depend
* add rosdep roscpp_tutorials

1.0.6
-----
* rtmros_common-1.0.6
* (hrpsys_tools, rosnode_rtc, rtmbuild) maintainer name adjusted to the one used in other places

1.0.5
-----
* "rtmros_common-1.0.5"

1.0.4
-----
* "rtmros_common-1.0.4"

1.0.3
-----
* "rtmros_common-1.0.3"

1.0.2
-----
* "rtmros_common-1.0.2"
* remove move_base_stage_tutorial since stage is not catkinzed in groovy

1.0.1
-----
* rtmros_common) 1.0.1
* rtmros_common) Revert wrong commit r5588
* rtmros_common) 1.0.1
* rtmros_common) Increment version to 1.0.0 (based on discussion https://code.google.com/p/rtm-ros-robotics/issues/detail?id=156&thanks=156&ts=1375860209).
* Updated all package.xml in rtmros_common metapkg (add URLs, alphabetized, cleanup, add Isaac as a maintainer to receive notice from buildfarm).
* catkinize rosnode_rtc
* inport_1.1.0 as default
* update to new openrtm_aist directory structure
* depends to openrtm_tools
* fix for groovy, this may breaks electric
* update for openrtm 1.1
* fixed index.rst and conf.py conflict problem
* add conf.py index.rst
* more verbose for rosnode_rtc example
* add a image in launchdocument
* update navigation.png
* add desktop icons
* add cut to remove space fro the rospack export ...
* add tkjoy_comp.sh  tkjoy_sample.launch
* fix layout
* quote backslash
* add 2 images, node-rtc detaport figure
* add newline in the end of rosdoc.yaml, and jsk_totoks depend
* changed sample launch to wait creating dataport before rtinject and rtprint
* add rosdoc.yaml
* add ignore_unbound mode, add documents and images
* added switch option to ignore unbounded message type
* changed to remove prefix ROS
* add parameter for rosnode wrapper
* add sleep after compiling idl
* add sample for connecting TkJoyStick
* add TkJoyStick -> CommandVel bridge sample
* added sample for OpenRTM-Python1.1.0, rtshell should be patched
* fixed the error check for creating ports
* fixed the determination of rtm data object class
* add simple usage in stage_sample.launch, add info for failure of port creation
* fixed the stage sample for send goal
* changed package name dataport_ros_bridge -> rosnode_rtc
* Contributors: Kei Okada, chen.jsk@gmail.com, gm130s@gmail.com, kei.okada, kei.okada@gmail.com, manabu1118, manabu1118@gmail.com
