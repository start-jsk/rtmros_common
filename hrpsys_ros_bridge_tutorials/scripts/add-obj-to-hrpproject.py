#!/usr/bin/env python

import os.path, sys, os, getopt
import subprocess
from xml.dom.minidom import parse, parseString
import xml.dom
import re
import string

#from xml.dom import minidom
#import sys
reload(sys)
sys.setdefaultencoding('utf-8')

#### >>> copied from xacro/src/xacro.py
# Better pretty printing of xml
# Taken from http://ronrothman.com/public/leftbraned/xml-dom-minidom-toprettyxml-and-silly-whitespace/
def fixed_writexml(self, writer, indent="", addindent="", newl=""):
    # indent = current indentation
    # addindent = indentation to add to higher levels
    # newl = newline string
    writer.write(indent+"<" + self.tagName)

    attrs = self._get_attributes()
    a_names = attrs.keys()
    a_names.sort()

    for a_name in a_names:
        writer.write(" %s=\"" % a_name)
        xml.dom.minidom._write_data(writer, attrs[a_name].value)
        writer.write("\"")
    if self.childNodes:
        if len(self.childNodes) == 1 \
          and self.childNodes[0].nodeType == xml.dom.minidom.Node.TEXT_NODE:
            writer.write(">")
            self.childNodes[0].writexml(writer, "", "", "")
            writer.write("</%s>%s" % (self.tagName, newl))
            return
        writer.write(">%s"%(newl))
        for node in self.childNodes:
            if node.nodeType is not xml.dom.minidom.Node.TEXT_NODE: # 3:
                node.writexml(writer,indent+addindent,addindent,newl) 
                #node.writexml(writer,indent+addindent,addindent,newl)
        writer.write("%s</%s>%s" % (indent,self.tagName,newl))
    else:
        writer.write("/>%s"%(newl))
# replace minidom's function with ours
xml.dom.minidom.Element.writexml = fixed_writexml
#### <<< copied from xacro/src/xacro.py

def add_attr(nd, name, value):
    attr = doc.createAttribute(name)
    attr.value = value
    nd.setAttributeNode(attr)
    return nd

def append_prop_node(pelem, name, value):
    nd = doc.createElement("property")
    nd = add_attr(nd, "name", name)
    nd = add_attr(nd, "value", value)
    pelem.appendChild(nd)
    return pelem

def append_item_node(pelem, objname, url, robot = False, translation = '0 0 0', rotation = '1 0 0 0'):
    nd = doc.createElement("item")
    nd = add_attr(nd, "class", "com.generalrobotix.ui.item.GrxModelItem")
    nd = add_attr(nd, "name", objname)
    nd = add_attr(nd, "select", "true")
    nd = add_attr(nd, "url", url)
    if robot:
        nd = append_prop_node(nd, "isRobot", "true")
    else:
        nd = append_prop_node(nd, "isRobot", "false")
    nd = append_prop_node(nd, "WAIST.rotation",    rotation)
    nd = append_prop_node(nd, "WAIST.translation", translation)

    pelem.appendChild(nd)
    return pelem

def append_collision_node(pelem, obj1, obj2):
    nd = doc.createElement("item")
    nd = add_attr(nd, "class", "com.generalrobotix.ui.item.GrxCollisionPairItem")
    nd = add_attr(nd, "name", "CP#%s_#%s_"%(obj1, obj2))
    nd = add_attr(nd, "select", "true")
    nd = append_prop_node(nd, "springConstant"   , "0 0 0 0 0 0")
    nd = append_prop_node(nd, "slidingFriction"  , "0.5")
    nd = append_prop_node(nd, "jointName1"       , "") ##
    nd = append_prop_node(nd, "jointName2"       , "") ##
    nd = append_prop_node(nd, "damperConstant"   , "0 0 0 0 0 0")
    nd = append_prop_node(nd, "objectName2"      , obj1)
    nd = append_prop_node(nd, "objectName1"      , obj2)
    nd = append_prop_node(nd, "springDamperModel", "false")
    nd = append_prop_node(nd, "staticFriction "  , "0.5")
    pelem.appendChild(nd)
    return pelem

def add_object_to_projectfile(objname, url, robot=False, add_collision=True, translation = '0 0 0', rotation = '1 0 0 0'):
    mode_elems = doc.getElementsByTagName('mode')
    # check mode size
    items = mode_elems[0].getElementsByTagName('item')
    item_list = []
    ## dolist items
    for item in items:
        if item.getAttribute('class') == "com.generalrobotix.ui.item.GrxModelItem":
            item_list.append (item.getAttribute('name'))

    #append_item_node(mode_elems[0], objname, url, robot)
    append_item_node(mode_elems[0], objname, url, robot, translation, rotation)

    if add_collision:
        for obj in item_list:
            append_collision_node(mode_elems[0], obj, objname)

## 
if __name__ == '__main__':
    global doc
    argvs = sys.argv
    argc = len(argvs)

    if argc < 4:
        print '### usage   : <in>.xml <out>.xml objname,url,is_robot_q,add_collision_q,0,0,0,0,0,0,0'
        print '### example : hoge.xml fuga.xml  table,url_of_table,True,True,0.0,0.0,1.0,0.0,0.0,1.0,90.0'
        print '###                             string,string      ,bool,bool,trans<3>   ,rotation<4>'
        exit(0)

    infile = argvs[1]
    outfile = argvs[2]
    objlist = argvs[3:]

    doc = xml.dom.minidom.parse(infile)

    for obj in objlist:
        ### parse add objects
        params = obj.split(',')
        #print params
        objname = params[0]
        url = params[1]
        robot = False
        if len(params) > 2:
            robot = bool(params[2])
        add_collision = False
        if len(params) > 3:
            add_collision = bool(params[3])
        translation = '0 0 0'
        if len(params) > 6:
            translation = "%s %s %s"%(params[4],params[5],params[6])
        rotation = '1 0 0 0'
        if len(params) > 10:
            rotation = "%s %s %s %s"%(params[7],params[8],params[9],params[10])

        add_object_to_projectfile(objname, url, robot, add_collision, translation, rotation)

    f = open(outfile, 'w')
    f.write(doc.toprettyxml(indent = '  '))
#    f.write(doc.toxml())
    f.close()
