#ifndef QUAZIP_QUAGZIPFILE_H
#define QUAZIP_QUAGZIPFILE_H

/*
Copyright (C) 2005-2014 Sergey A. Tachenov

This file is part of QuaZIP.

QuaZIP is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 2.1 of the License, or
(at your option) any later version.

QuaZIP is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with QuaZIP.  If not, see <http://www.gnu.org/licenses/>.

See COPYING file for the full LGPL text.

Original ZIP package is copyrighted by Gilles Vollant and contributors,
see quazip/(un)zip.h files for details. Basically it's the zlib license.
*/

#include <QIODevice>
#include "quazip_global.h"

#include <zlib.h>

class QuaGzipFilePrivate;

/// GZIP file
/**
  This class is a wrapper around GZIP file access functions in zlib. Unlike QuaZip classes, it doesn't allow reading from a GZIP file opened as QIODevice, for example, if your GZIP file is in QBuffer. It only provides QIODevice access to a GZIP file contents, but the GZIP file itself must be identified by its name on disk or by descriptor id.
  */
class QUAZIP_EXPORT QuaGzipFile: public QIODevice {
  Q_OBJECT
public:
  /// Empty constructor.
  /**
    Must call setFileName() before trying to open.
    */
  QuaGzipFile();
  /// Empty constructor with a parent.
  /**
    Must call setFileName() before trying to open.
    \param parent The parent object, as per QObject logic.
    */
  QuaGzipFile(QObject *parent);
  /// Constructor.
  /**
    \param fileName The name of the GZIP file.
    \param parent The parent object, as per QObject logic.
    */
  QuaGzipFile(const QString &fileName, QObject *parent = NULL);
  /// Destructor.
  virtual ~QuaGzipFile();
  /// Sets the name of the GZIP file to be opened.
  void setFileName(const QString& fileName);
  /// Returns the name of the GZIP file.
  QString getFileName() const;
  /// Returns true.
  /**
    Strictly speaking, zlib supports seeking for GZIP files, but it is
    poorly implemented, because there is no way to implement it
    properly. For reading, seeking backwards is very slow, and for
    writing, it is downright impossible. Therefore, QuaGzipFile does not
    support seeking at all.
    */
  virtual bool isSequential() const;
  /// Opens the file.
  /**
    \param mode Can be either QIODevice::Write or QIODevice::Read.
    ReadWrite and Append aren't supported.
    */
  virtual bool open(QIODevice::OpenMode mode);
  /// Opens the file.
  /**
    \overload
    \param fd The file descriptor to read/write the GZIP file from/to.
    \param mode Can be either QIODevice::Write or QIODevice::Read.
    ReadWrite and Append aren't supported.
    */
  virtual bool open(int fd, QIODevice::OpenMode mode);
  /// Flushes data to file.
  /**
    The data is written using Z_SYNC_FLUSH mode. Doesn't make any sense
    when reading.
    */
  virtual bool flush();
  /// Closes the file.
  virtual void close();
protected:
  /// Implementation of QIODevice::readData().
  virtual qint64 readData(char *data, qint64 maxSize);
  /// Implementation of QIODevice::writeData().
  virtual qint64 writeData(const char *data, qint64 maxSize);
private:
    // not implemented by design to disable copy
    QuaGzipFile(const QuaGzipFile &that);
    QuaGzipFile& operator=(const QuaGzipFile &that);
    QuaGzipFilePrivate *d;
};

#endif // QUAZIP_QUAGZIPFILE_H
