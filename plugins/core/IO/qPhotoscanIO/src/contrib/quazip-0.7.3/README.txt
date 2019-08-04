QuaZIP is the C++ wrapper for Gilles Vollant's ZIP/UNZIP package
(AKA minizip) using Trolltech's Qt library.

It uses existing ZIP/UNZIP package C code and therefore depends on
the zlib library.

Also, it depends on Qt 4.

To compile it on UNIX dialect:

$ cd quazip
$ qmake
$ make

You must make sure that:
* You have Qt 4 properly and fully installed (including tools and
  headers, not just library)
* "qmake" command runs Qt 4's qmake, not some other version (you'll have
  to type full path to qmake otherwise).

To install compiled shared library, just type:

$ make install

By default, it installs in /usr/local, but you may change it using

$ qmake PREFIX=/wherever/you/want/to/install

You do not have to compile and install QuaZIP to use it. You can just
(and sometimes it may be the best way) add QuaZIP's source files to your
project and use them.

See doc/html or, if you do not have a browser, quazip/*.h and
quazip/doc/* files for the more detailed documentation.

For Windows, it's essentially the same, but you may have to adjust
settings for different environments.

If linking statically (either a static lib or just using the source code
directly in your project), then QUAZIP_STATIC should be defined. This is
done automatically when you build QuaZIP as a static library. However,
when _using_ a static lib (or source code, for that matter) you must
also define QUAZIP_STATIC in your project (that uses QuaZIP) to tell
quazip_global.h that you use a static version because otherwise the
compiler wouldn't know that and will mark QuaZIP symbols as dllimported.
Linking problems among the lines of “undefined reference” are usually
caused by this.

Copyright notice:

Copyright (C) 2005-2012 Sergey A. Tachenov

This program is free software; you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 2 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

See COPYING file for the full LGPL text.

Original ZIP package is copyrighted by Gilles Vollant, see
quazip/(un)zip.h files for details, basically it's zlib license.
