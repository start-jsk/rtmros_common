#!/bin/bash

TEST_DIR=`rospack find openhrp3`/test

cat <<EOF > $TEST_DIR/sample-projects.rst
OpenHPR3 examples
=================
EOF

for filename in `rospack find openhrp3`/share/OpenHRP-3.1/sample/project/*.xml
do
    if [ -f project-`basename $filename .xml`.png ]; then
	cat <<EOF >> $TEST_DIR/sample-projects.rst
`basename $filename .xml`
-------------------------

.. code-block:: bash

  roscd openhrp3/share/OpenHRP-3.1/sample/project/
  rosrun openhrp3 grxui.sh `basename $filename`

.. image :: project-`basename $filename .xml`.png
    :width: 500pt
EOF
	else
	cat <<EOF >> $TEST_DIR/sample-projects.rst
`basename $filename .xml` is not tested
=======================================

EOF
	fi
	cat <<EOF >> $TEST_DIR/sample-projects.rst

Download \``basename $filename`\`_ file

.. _\``basename $filename`\`: ../../share/OpenHRP-3.1/sample/project/`basename $filename`

-------------------

EOF

done

sphinx-build  -b html -d `rospack find openhrp3`/build/doctrees $TEST_DIR `rospack find openhrp3`/build/html
