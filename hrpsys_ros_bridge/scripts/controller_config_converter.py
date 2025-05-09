#!/usr/bin/env python
import yaml
import sys

if __name__ == '__main__':
    if len(sys.argv) < 1:
        print('usage : %s [ input.yaml ] output.yaml'%sys.argv[0])


    if len(sys.argv) > 2:
        inputfile = sys.argv[1]
        outputfile = sys.argv[2]
    else:
        #outputfile = inputfile.replace('.yaml', '') + '_controller_config.yaml'
        inputfile = ""
        outputfile = sys.argv[1]

    of = open(outputfile, 'w')

    print('##', file=of)
    print('## auto generated file', file=of)
    print('##', file=of)
    print('##controller_configuration:', file=of)
    print('##  - group_name: <joint group name>', file=of)
    print('##    controller_name: <name of joint trajectory topic name>', file=of)
    print('##    joint_list: ## list of using joints', file=of)
    print('##      - <joint_name>', file=of)

    if inputfile == "":
        of.close()
        exit(0)

    lst = yaml.load(open(inputfile).read())

    print('controller_configuration:', file=of)

    for limb in list(lst.keys()):
        if limb == 'sensors':
            continue
        if limb.endswith('-coords') or limb.endswith('-vector'):
            continue
        if limb == 'links' or limb == 'replace_xmls':
            continue
        jlst = [list(j.keys())[0] for j in lst[limb] if isinstance(j, dict) and isinstance(list(j.values())[0], str)]
        if len(jlst) > 0:
            print('  - group_name: ' + limb, file=of)
            print('    controller_name: /' + limb + '_controller', file=of)
            print('    joint_list:', file=of)
            for j in jlst:
                print('      - ' + j, file=of)

    of.close()
    exit(0)
