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


TEST_DIR=`rospack find openhrp3`/test
rev=`LANG=C svn info \`rospack find openhrp3\` | grep ^Revision`
echo "OpenHPR3 examples ($rev)" > $TEST_DIR/index.rst
echo "==========================" >> $TEST_DIR/index.rst

for filename in `rospack find openhrp3`/share/OpenHRP-3.1/sample/project/*.xml
do
    if [ -f $TEST_DIR/project-`basename $filename .xml`.png ]; then
	cat <<EOF >> $TEST_DIR/index.rst
`basename $filename .xml`
-------------------------

.. code-block:: bash

  roscd openhrp3/share/OpenHRP-3.1/sample/project/
  rosrun openhrp3 grxui.sh `basename $filename`

.. image :: project-`basename $filename .xml`.png
    :width: 500pt
EOF
	else
	cat <<EOF >> $TEST_DIR/index.rst
`basename $filename .xml` (not tested)
--------------------------------------

EOF
	fi
	cat <<EOF >> $TEST_DIR/index.rst

Download \``basename $filename`\`_ file

.. _\``basename $filename`\`: ../../share/OpenHRP-3.1/sample/project/`basename $filename`

-------------------

EOF

done

sphinx-build  -b singlehtml -d `rospack find openhrp3`/build/doctrees $TEST_DIR `rospack find openhrp3`/build/html
