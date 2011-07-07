include $(shell rospack find mk)/cmake.mk

wipe: clean
	make -f Makefile.openhrp-aist-grx wipe
	touch wiped

clean:
	make -f Makefile.openhrp-aist-grx clean

eclipse-clean:
	-sudo rm /usr/lib/eclipse/plugins/com.generalrobotix.*
	-sudo rm /usr/lib/eclipse/plugins/jp.go.aist.rtm.*
	-rm -fr ~/workspace `rospack find openhrp3`/workspace ~/.eclipse



