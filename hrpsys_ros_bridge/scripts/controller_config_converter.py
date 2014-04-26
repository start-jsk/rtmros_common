#!/usr/bin/env python
import yaml
import sys

if __name__ == '__main__':
    if len(sys.argv) < 1:
        print 'usage : %s [ input.yaml ] output.yaml'%sys.argv[0]


    if len(sys.argv) > 2:
        inputfile = sys.argv[1]
        outputfile = sys.argv[2]
    else:
        #outputfile = inputfile.replace('.yaml', '') + '_controller_config.yaml'
        inputfile = ""
        outputfile = sys.argv[1]

    of = open(outputfile, 'w')

    print >> of, '##'
    print >> of, '## auto generated file'
    print >> of, '##'
    print >> of, '##controller_configuration:'
    print >> of, '##  - group_name: <joint group name>'
    print >> of, '##    controller_name: <name of joint trajectory topic name>'
    print >> of, '##    joint_list: ## list of using joints'
    print >> of, '##      - <joint_name>'

    if inputfile == "":
        of.close()
        exit(0)

    lst = yaml.load(open(inputfile).read())

    limb_candidates = ('larm', 'rarm', 'lleg', 'rleg', 'torso', 'head') ## candidates of limb names

    invalid_yaml = True
    for l in lst.keys():
        if l in limb_candidates:
            invalid_yaml = False
            break

    if invalid_yaml:
        of.close()
        exit(0)

    print >> of, 'controller_configuration:'

    for limb in limb_candidates:
        #    endcoords_name = limb+'-end-coords'
        #    endcoords_info = lst[endcoords_name]
        if limb in lst.keys():
            jlst = [j.keys()[0] for j in lst[limb]]
            if len(jlst) > 0:
                print >> of, '  - group_name: ' + limb
                print >> of, '    controller_name: /' + limb + '_controller'
                print >> of, '    joint_list:'
                for j in jlst:
                    print >> of, '      - ' + j

    of.close()
    exit(0)
