Ball: installed java3d plugin_archives all
include $(shell rospack find mk)/cmake.mk

include Makefile.openhrp3

check-java-version:
	@if [ ! "`java -version 2>&1 | grep Java\(TM\)\ SE`" ]; then \
	   /bin/echo -e "\e[1;34mSwitch java runtime to to Sun (TM) SE\e[m"; \
	   sudo update-java-alternatives -s java-6-sun; \
	fi

check-eclipse-launch:
	if [ `lsb_release -r -s | cut -d '.' -f 1` -ge 12 ]; then \
           if [ "`grep 'java.library.path=/usr/lib/jni' /etc/eclipse.ini`" = "" ]; then \
              /bin/echo "-Djava.library.path=/usr/lib/jni" | sudo bash -c 'cat >> /etc/eclipse.ini'; \
           fi ; \
        fi

installed: installed.openhrp3

clean: clean.openhrp3

eclipse-clean:
	-sudo rm /usr/lib/eclipse/plugins/com.generalrobotix.*
	-sudo rm /usr/lib/eclipse/plugins/jp.go.aist.rtm.*
	-rm -fr ~/workspace `rospack find openhrp3`/workspace ~/.eclipse



