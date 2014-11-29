TMPDIR := $(shell mktemp -d)

all: rtmros_common_manual.pdf html

copy_eus_tex:
	mkdir -p ${TMPDIR}
	cp rtmros_common_manual.tex ${TMPDIR}

rtmros_common_manual.pdf: copy_eus_tex
	(cd ${TMPDIR};\
	roseus ../eus/lib/llib/documentation.l "(progn\
	(load \"package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l\")\
	(make-document (format nil \"~A/euslisp/rtm-ros-robot-interface.l\" (ros::resolve-ros-path \"package://hrpsys_ros_bridge\")) \"rtm-ros-robot-interface-func.tex\")\
	(exit))")
	nkf --in-place -e ${TMPDIR}/*.tex;
	(cd ${TMPDIR}; platex rtmros_common_manual; dvipdfmx rtmros_common_manual.dvi)

open:
	gnome-open rtmros_common_manual.pdf
	gnome-open html/index.html

html: rtmros_common_manual.pdf
	nkf --in-place -u ${TMPDIR}/*.tex
	mkdir ${TMPDIR}/html
	sed -i 's@^\(\\\usepackage.*{hyperref}\)@%\1@' ${TMPDIR}/rtmros_common_manual.tex
	sed -i 's@\\\documentclass\[\]{jarticle}@\\\documentclass\[\]{article}@' ${TMPDIR}/rtmros_common_manual.tex
	sed -i 's@^\\\ifnum 42146@%\\\ifnum 42146@' ${TMPDIR}/rtmros_common_manual.tex
	(cd ${TMPDIR}; charset="UTF-8" latex2html -dir ${TMPDIR}/html -local_icons -auto_prefix -iso_language JP rtmros_common_manual)
	(cd ${TMPDIR}/html; sed -i 's@</TITLE>@</TITLE>\n<META HTTP-EQUIV="Content-Type" CONTENT="text/html; charset=UTF-8">@' *.html index.html)
	sed -i 's@%\(\\\usepackage.*{hyperref}\)@\1@' ${TMPDIR}/rtmros_common_manual.tex
	sed -i 's@\\\documentclass\[\]{article}@\\\documentclass\[\]{jarticle}@' ${TMPDIR}/rtmros_common_manual.tex
	sed -i 's@^%\\\ifnum 42146@\\\ifnum 42146@' ${TMPDIR}/rtmros_common_manual.tex
	cp -r ${TMPDIR}/html ./
	cp ${TMPDIR}/rtmros_common_manual.pdf ./html/
	rm -fr ${TMPDIR}

clean:
	rm -f *.log *.out *.toc *.dvi *.idx *.aux *.pdf
	rm -fr html ${TMPDIR}
