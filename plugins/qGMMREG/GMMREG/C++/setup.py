#!/usr/bin/env python
#coding=utf-8

#======================================================================
#$RCSfile: setup.py,v $
#$Author: bing.jian $
#$Date: 2009-02-10 02:13:49 -0500 (Tue, 10 Feb 2009) $
#$Revision: 121 $
#======================================================================


from distutils.core import setup, Extension

pygmmreg = Extension('pygmmreg',
                     define_macros = [('MAJOR_VERSION', '1'),
                                      ('MINOR_VERSION', '0')],
                     include_dirs = ['/usr/include'],
                     libraries = ['gmmreg_api'],
                     library_dirs = ['/usr/lib64','./build'],
                     sources = ['pygmmreg.cpp'])

setup (name = 'pygmmreg',
              version = '1.0',
              description = 'Python wrapper of GMMREG algorithms.',
              author = 'Bing Jian',
              author_email = 'bing.jian@gmail.com',
              url = 'http://www.python.org/doc/current/ext/building.html',
              long_description = '''
              This is a Python wrapper of the C++ implementation of GMMREG algorithms.
              ''',
              ext_modules = [pygmmreg])
