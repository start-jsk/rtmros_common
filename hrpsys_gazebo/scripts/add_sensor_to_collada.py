#!/usr/bin/env python

import sys, os
from xml.dom.minidom import parse, parseString
import xml.dom

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

## kinematics_model -> id
def add_sensor (doc, name, parent_link, sensor_type, translate = None, rotate = None):
    global sensor_num, library_sensors_node, target_articulated_system
    global force_sensor_num, gyro_sensor_num, acc_sensor_num

    ### add sensor to articulated system
    ex = doc.createElement('extra')
    ex.setAttribute('name', name)
    ex.setAttribute('type', 'attach_sensor')

    tec = doc.createElement('technique')
    tec.setAttribute('profile', 'OpenRAVE')

    tmp = doc.createElement('instance_sensor')
    tmp.setAttribute('url', '#sensor%d' % sensor_num)
    sensor_num = sensor_num + 1
    tec.appendChild(tmp)

    tmp = doc.createElement('frame_origin')
    tmp.setAttribute('link', 'kmodel0/%s'%parent_link)
    if translate == None:
        tl = doc.createElement('translate')
        tl.appendChild(doc.createTextNode('0 0 0'))
    else:
        tl = doc.createElement('translate')
        tl.appendChild(doc.createTextNode(translate))

    if rotate == None:
        ro = doc.createElement('rotate')
        ro.appendChild(doc.createTextNode('1 0 0 0'))
    else:
        ro = doc.createElement('rotate')
        ro.appendChild(doc.createTextNode(rotate))

    tmp.appendChild(tl)
    tmp.appendChild(ro)

    tec.appendChild(tmp)
    ex.appendChild(tec)
    target_articulated_system.appendChild(ex)

    ### add to library_sensors
    plst = library_sensors_node.getElementsByTagName('technique')
    targetNode = None
    for p in plst:
        if p.hasAttribute('profile') and p.getAttribute('profile') == 'OpenRAVE':
            targetNode = p

    if targetNode != None:
        sen = doc.createElement('sensor')
        sen.setAttribute('id', 'sensor%d' % (sensor_num - 1))

        if sensor_type == 'force':
            sen.setAttribute('type', 'base_force6d')
            sen.setAttribute('sid', '%d'%force_sensor_num)
            force_sensor_num = force_sensor_num + 1
            lf = doc.createElement('load_range_force')
            lf.appendChild(doc.createTextNode('-1.0 -1.0 -1.0'))
            lt = doc.createElement('load_range_torque')
            lt.appendChild(doc.createTextNode('-1.0 -1.0 -1.0'))
            sen.appendChild(lf)
            sen.appendChild(lt)
        elif sensor_type == 'acceleration':
            sen.setAttribute('type', 'base_imu')
            sen.setAttribute('sid', '%d'%acc_sensor_num)
            acc_sensor_num = acc_sensor_num + 1
            max_acc = doc.createElement('max_acceleration')
            max_acc.appendChild(doc.createTextNode('-1.0 -1.0 -1.0'))
            sen.appendChild(max_acc)
        elif sensor_type == 'gyro':
            sen.setAttribute('type', 'base_imu')
            sen.setAttribute('sid', '%d'%gyro_sensor_num)
            gyro_sensor_num = gyro_sensor_num + 1
            max_ang = doc.createElement('max_angular_velocity')
            max_ang.appendChild(doc.createTextNode('-1.0 -1.0 -1.0'))
            sen.appendChild(max_ang)

        targetNode.appendChild(sen)

if __name__ == '__main__':
    global sensor_num, library_sensors_node, target_articulated_system
    global force_sensor_num, gyro_sensor_num, acc_sensor_num
    argvs = sys.argv
    argc = len(argvs)
    if argc > 1:
        fname = argvs[1]
    else:
        print 'usage: %s intput_filename > outputfile'
        exit(0)

    sensor_num = 1
    force_sensor_num = 0
    gyro_sensor_num = 0
    acc_sensor_num = 0

    doc = xml.dom.minidom.parse(fname)

    plst = doc.getElementsByTagName('extra')
    library_sensors_node = None
    for p in plst:
        if p.hasAttribute('id') and p.getAttribute('id') == 'sensors' and p.hasAttribute('type') and p.getAttribute('type') == 'library_sensors':
            library_sensors_node = p

    plst = doc.getElementsByTagName('articulated_system')
    target_articulated_system = None
    for p in plst:
        if p.hasAttribute('id') and p.getAttribute('id') == 'robot0_motion':
            target_articulated_system = p

    if library_sensors_node != None and target_articulated_system != None:
        ## now added sensors are hard coded....
        add_sensor(doc, 'lhsensor', 'l_hand', 'force')
        add_sensor(doc, 'rhsensor', 'r_hand', 'force')
        add_sensor(doc, 'lfsensor', 'l_foot', 'force')
        add_sensor(doc, 'rfsensor', 'r_foot', 'force')
        add_sensor(doc, 'gsensor', 'imu_link', 'acceleration')
        add_sensor(doc, 'gyrometer', 'imu_link', 'gyro')
        #add_sensor(doc, 'lgsensor', 'l_hand', 'acceleration')
        #add_sensor(doc, 'lgyrometer', 'l_hand', 'gyro')
        #add_sensor(doc, 'rgsensor', 'r_hand', 'acceleration')
        #add_sensor(doc, 'rgyrometer', 'r_hand', 'gyro')
        #add_sensor(doc, 'hgsensor', 'head_imu_link', 'acceleration')
        #add_sensor(doc, 'hgyrometer', 'head_imu_link', 'gyro')
        sys.stdout.write(doc.toprettyxml(indent = '\t'))
