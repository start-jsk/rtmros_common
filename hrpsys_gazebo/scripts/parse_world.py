#!/usr/bin/env python

from xml.dom import minidom
import os
import urllib
import urlparse
import sys

reload(sys)
sys.setdefaultencoding('utf-8')


def search_gazebo_dir (dirname, gpath = ['.']):
    for pdir in gpath:
        if os.access(pdir+'/'+dirname, os.F_OK):
            return pdir+'/'+dirname

def resolve_gazebo_path (fname):
    if fname.find('model://') >= 0:
        up = urlparse.urlparse(fname)
        dname = up.netloc
        try:
            dirs = os.environ['GAZEBO_MODEL_PATH'].split(':')
            dirs.append('/tmp')
        except:
            print ""
            return fname
        gp = search_gazebo_dir (dname, gpath = dirs)
        if gp:
            return gp
        else:
            print ""
            return fname
    elif fname.find('file://media') >= 0:
        up = urlparse.urlparse(fname)
        dname = up.netloc + up.path
        try:
            dirs = os.environ['GAZEBO_RESOURCE_PATH'].split(':')
        except:
            print ""
            return fname
        gp = search_gazebo_dir (dname, gpath = dirs)
        if gp:
            return gp
        else:
            print ""
            return fname
    elif fname.find('package://') >= 0:
        return
    return fname

def check_model_url (model):
    model = resolve_gazebo_path(model)
    pos = model.find('model://')
    if pos == -1:
        return model
    modelname = model[pos+8:] ## remove model://
    sys.stderr.write(";; model = %s\n"%modelname)
    tmpmodel = '/tmp/%s' % modelname
    if os.access(tmpmodel, os.F_OK):
        sys.stderr.write(";; at TMP: %s\n"%modelname)
        return tmpmodel
    fp = urllib.urlopen("http://gazebosim.org/models/%s/model.tar.gz" % modelname)
    if fp.getcode() == 200:
        f = open("/tmp/model.tar.gz", 'wb')
        f.write(fp.read())
        fp.close()
        f.close()
        os.system('tar xf /tmp/model.tar.gz -C /tmp')
        if os.access(tmpmodel, os.F_OK):
            sys.stderr.write(";; create at TMP: %s\n"%modelname)
            return tmpmodel
    return model

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
        wrd_nm = wrd[0].getAttribute('name')
        print "(list"
        for inc in wrd[0].getElementsByTagName('include'):
            pose = ''
            name = ''
            modelf = ''

            elem = inc.getElementsByTagName('uri')
            if elem:
                modelf = getText(elem[0].childNodes)
                modelf =  check_model_url(modelf)

            elem = inc.getElementsByTagName('pose')
            if elem:
                pose = getText(elem[0].childNodes)

            elem = inc.getElementsByTagName('name')
            if elem:
                name = getText(elem[0].childNodes)

            print "(list (list :model \"%s\")" % modelf
            if pose != '':
                print "      (list :pose \"%s\")" % pose
            if name != '':
                print "      (list :name \"%s\")" % name
            print ")"

        for mdl in wrd[0].getElementsByTagName('model'):
            nm = mdl.getAttribute('name')
            if nm:
                nm = nm + '_%s'%wrd_nm
                dirname = '/tmp/%s' % nm
                if not os.access(dirname, os.F_OK):
                    os.makedirs(dirname)
                f = open('%s/model.sdf' % dirname, 'wb')
                f.write('<?xml version="1.0"?>\n<sdf version="1.3">\n')
                f.write(mdl.toxml())
                f.write('</sdf>')
                f.close()
                print '(list (list :model \"%s\")' % dirname
                pose = getText(mdl.getElementsByTagName('pose')[0].childNodes)
                if pose:
                    print '      (list :pose  \"%s\")' % pose
                print ')'
        print ")"
