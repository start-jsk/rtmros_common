#!/usr/bin/env python

from xml.dom import minidom
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

def getText(nodelist):
    rc = []
    for node in nodelist:
        if node.nodeType == node.TEXT_NODE:
            rc.append(node.data)
    return ''.join(rc)

if __name__ == '__main__':
    argvs = sys.argv
    argc = len(argvs)

    global doc, worldf
    if argc > 1:
        worldf = argvs[1]
    else:
        exit(0)

    doc = minidom.parse(worldf)

    wrd = doc.getElementsByTagName('world')

    if wrd != []:
        print "(list"
        for inc in wrd[0].getElementsByTagName('include'):
            modelf = getText(inc.getElementsByTagName('uri')[0].childNodes)
            pose = getText(inc.getElementsByTagName('pose')[0].childNodes)
            print "(list (list :model \"%s\")" % modelf
            if pose != '':
                print "      (list :pose \"%s\")" % pose
            print ")"
        print ")"
