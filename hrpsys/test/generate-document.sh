#!/bin/bash -x

if [ $# -gt 0 ] ; then
    GETOPT=`getopt -l gtest_output:,text -- dummy "$@"` ; [ $? != 0 ] && exit 1
    eval set -- "$GETOPT"
    while true
    do
	case $1 in
	    --gtest_output)  TEST_OUTPUT=`echo $2|sed s/^xml\://`     ; shift 2
		;;
	    --text)  shift 1
		;;
	    --)  shift; break;
		;;
	esac
    done
    if [ "$TEST_OUTPUT" != "" ] ; then
	touch $TEST_OUTPUT # for dummy gtest output
    fi
fi


TEST_DIR=`rospack find hrpsys`/test
rev=`LANG=C svn info \`rospack find hrpsys\` | grep ^Revision`
echo "hrpsys examples ($rev)" > $TEST_DIR/index.rst
echo "==========================" >> $TEST_DIR/index.rst

for filename in `rospack find hrpsys`/launch/*.launch
do
    if [ -f $TEST_DIR/`basename $filename .launch`-grxui.png ]; then
	convert -delay 10 -loop 0 $TEST_DIR/`basename $filename .launch`-grxui-*.png $TEST_DIR/`basename $filename .launch`-grxui.gif
	cat <<EOF >> $TEST_DIR/index.rst
`basename $filename .launch`
-------------------------

.. code-block:: bash

  roslaunch hrpsys `basename $filename`

.. image :: `basename $filename .launch`-grxui.gif
    :width: 500pt
EOF
	else
	cat <<EOF >> $TEST_DIR/index.rst
`basename $filename .launch` (not tested)
--------------------------------------

EOF
	fi
	cat <<EOF >> $TEST_DIR/index.rst

Download \``basename $filename`\`_ file

.. _\``basename $filename`\`: ../../launch/`basename $filename`

-------------------

EOF

done

sphinx-build  -b singlehtml -d `rospack find hrpsys`/build/doctrees $TEST_DIR `rospack find hrpsys`/build/html

