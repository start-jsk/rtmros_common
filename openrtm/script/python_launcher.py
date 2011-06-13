#!/usr/bin/env python
import os,sys
from optparse import OptionParser

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('--file','-f',action='store',type='string',dest='file',default=None,help='set absolute path for executive file')
    (options, args) = parser.parse_args()
    if options.file is None:
        print 'please set file path'
    else:
        os.chdir(os.path.dirname(options.file))
        os.system("python " + os.path.basename(options.file))

