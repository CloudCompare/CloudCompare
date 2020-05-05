/* ioapi.c -- IO base function header for compress/uncompress .zip
   files using zlib + zip or unzip API

   Version 1.01e, February 12th, 2005

   Copyright (C) 1998-2005 Gilles Vollant

   Modified by Sergey A. Tachenov to integrate with Qt.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "zlib.h"
#include "ioapi.h"
#include "quazip_global.h"
#include <QIODevice>
#if (QT_VERSION >= 0x050100)
#define QUAZIP_QSAVEFILE_BUG_WORKAROUND
#endif
#ifdef QUAZIP_QSAVEFILE_BUG_WORKAROUND
#include <QSaveFile>
#endif

/* I've found an old Unix (a SunOS 4.1.3_U1) without all SEEK_* defined.... */

#ifndef SEEK_CUR
#define SEEK_CUR    1
#endif

#ifndef SEEK_END
#define SEEK_END    2
#endif

#ifndef SEEK_SET
#define SEEK_SET    0
#endif

voidpf call_zopen64 (const zlib_filefunc64_32_def* pfilefunc,voidpf file,int mode)
{
    if (pfilefunc->zfile_func64.zopen64_file != NULL)
        return (*(pfilefunc->zfile_func64.zopen64_file)) (pfilefunc->zfile_func64.opaque,file,mode);
    else
    {
        return (*(pfilefunc->zopen32_file))(pfilefunc->zfile_func64.opaque,file,mode);
    }
}

int call_zseek64 (const zlib_filefunc64_32_def* pfilefunc,voidpf filestream, ZPOS64_T offset, int origin)
{
    if (pfilefunc->zfile_func64.zseek64_file != NULL)
        return (*(pfilefunc->zfile_func64.zseek64_file)) (pfilefunc->zfile_func64.opaque,filestream,offset,origin);
    else
    {
        uLong offsetTruncated = (uLong)offset;
        if (offsetTruncated != offset)
            return -1;
        else
            return (*(pfilefunc->zseek32_file))(pfilefunc->zfile_func64.opaque,filestream,offsetTruncated,origin);
    }
}

ZPOS64_T call_ztell64 (const zlib_filefunc64_32_def* pfilefunc,voidpf filestream)
{
    if (pfilefunc->zfile_func64.zseek64_file != NULL)
        return (*(pfilefunc->zfile_func64.ztell64_file)) (pfilefunc->zfile_func64.opaque,filestream);
    else
    {
        uLong tell_uLong = (*(pfilefunc->ztell32_file))(pfilefunc->zfile_func64.opaque,filestream);
        if ((tell_uLong) == ((uLong)-1))
            return (ZPOS64_T)-1;
        else
            return tell_uLong;
    }
}

/// @cond internal
struct QIODevice_descriptor {
    // Position only used for writing to sequential devices.
    qint64 pos;
    inline QIODevice_descriptor():
        pos(0)
    {}
};
/// @endcond

voidpf ZCALLBACK qiodevice_open_file_func (
   voidpf opaque,
   voidpf file,
   int mode)
{
    QIODevice_descriptor *d = reinterpret_cast<QIODevice_descriptor*>(opaque);
    QIODevice *iodevice = reinterpret_cast<QIODevice*>(file);
    QIODevice::OpenMode desiredMode;
    if ((mode & ZLIB_FILEFUNC_MODE_READWRITEFILTER)==ZLIB_FILEFUNC_MODE_READ)
        desiredMode = QIODevice::ReadOnly;
    else if (mode & ZLIB_FILEFUNC_MODE_EXISTING)
        desiredMode = QIODevice::ReadWrite;
    else if (mode & ZLIB_FILEFUNC_MODE_CREATE)
        desiredMode = QIODevice::WriteOnly;
    if (iodevice->isOpen()) {
        if ((iodevice->openMode() & desiredMode) == desiredMode) {
            if (desiredMode != QIODevice::WriteOnly
                    && iodevice->isSequential()) {
                // We can use sequential devices only for writing.
                delete d;
                return NULL;
            } else {
                if ((desiredMode & QIODevice::WriteOnly) != 0) {
                    // open for writing, need to seek existing device
                    if (!iodevice->isSequential()) {
                        iodevice->seek(0);
                    } else {
                        d->pos = iodevice->pos();
                    }
                }
            }
            return iodevice;
        } else {
            delete d;
            return NULL;
        }
    }
    iodevice->open(desiredMode);
    if (iodevice->isOpen()) {
        if (desiredMode != QIODevice::WriteOnly && iodevice->isSequential()) {
            // We can use sequential devices only for writing.
            iodevice->close();
            delete d;
            return NULL;
        } else {
            return iodevice;
        }
    } else {
        delete d;
        return NULL;
    }
}


uLong ZCALLBACK qiodevice_read_file_func (
   voidpf opaque,
   voidpf stream,
   void* buf,
   uLong size)
{
    QIODevice_descriptor *d = reinterpret_cast<QIODevice_descriptor*>(opaque);
    QIODevice *iodevice = reinterpret_cast<QIODevice*>(stream);
    qint64 ret64 = iodevice->read((char*)buf,size);
    uLong ret;
    ret = (uLong) ret64;
    if (ret64 != -1) {
        d->pos += ret64;
    }
    return ret;
}


uLong ZCALLBACK qiodevice_write_file_func (
   voidpf opaque,
   voidpf stream,
   const void* buf,
   uLong size)
{
    QIODevice_descriptor *d = reinterpret_cast<QIODevice_descriptor*>(opaque);
    QIODevice *iodevice = reinterpret_cast<QIODevice*>(stream);
    uLong ret;
    qint64 ret64 = iodevice->write((char*)buf,size);
    if (ret64 != -1) {
        d->pos += ret64;
    }
    ret = (uLong) ret64;
    return ret;
}

uLong ZCALLBACK qiodevice_tell_file_func (
   voidpf opaque,
   voidpf stream)
{
    QIODevice_descriptor *d = reinterpret_cast<QIODevice_descriptor*>(opaque);
    QIODevice *iodevice = reinterpret_cast<QIODevice*>(stream);
    uLong ret;
    qint64 ret64;
    if (iodevice->isSequential()) {
        ret64 = d->pos;
    } else {
        ret64 = iodevice->pos();
    }
    ret = static_cast<uLong>(ret64);
    return ret;
}

ZPOS64_T ZCALLBACK qiodevice64_tell_file_func (
   voidpf opaque,
   voidpf stream)
{
    QIODevice_descriptor *d = reinterpret_cast<QIODevice_descriptor*>(opaque);
    QIODevice *iodevice = reinterpret_cast<QIODevice*>(stream);
    qint64 ret;
    if (iodevice->isSequential()) {
        ret = d->pos;
    } else {
        ret = iodevice->pos();
    }
    return static_cast<ZPOS64_T>(ret);
}

int ZCALLBACK qiodevice_seek_file_func (
   voidpf /*opaque UNUSED*/,
   voidpf stream,
   uLong offset,
   int origin)
{
    QIODevice *iodevice = reinterpret_cast<QIODevice*>(stream);
    if (iodevice->isSequential()) {
        if (origin == ZLIB_FILEFUNC_SEEK_END
                && offset == 0) {
            // sequential devices are always at end (needed in mdAppend)
            return 0;
        } else {
            qWarning("qiodevice_seek_file_func() called for sequential device");
            return -1;
        }
    }
    uLong qiodevice_seek_result=0;
    int ret;
    switch (origin)
    {
    case ZLIB_FILEFUNC_SEEK_CUR :
        qiodevice_seek_result = ((QIODevice*)stream)->pos() + offset;
        break;
    case ZLIB_FILEFUNC_SEEK_END :
        qiodevice_seek_result = ((QIODevice*)stream)->size() - offset;
        break;
    case ZLIB_FILEFUNC_SEEK_SET :
        qiodevice_seek_result = offset;
        break;
    default:
        return -1;
    }
    ret = !iodevice->seek(qiodevice_seek_result);
    return ret;
}

int ZCALLBACK qiodevice64_seek_file_func (
   voidpf /*opaque UNUSED*/,
   voidpf stream,
   ZPOS64_T offset,
   int origin)
{
    QIODevice *iodevice = reinterpret_cast<QIODevice*>(stream);
    if (iodevice->isSequential()) {
        if (origin == ZLIB_FILEFUNC_SEEK_END
                && offset == 0) {
            // sequential devices are always at end (needed in mdAppend)
            return 0;
        } else {
            qWarning("qiodevice_seek_file_func() called for sequential device");
            return -1;
        }
    }
    qint64 qiodevice_seek_result=0;
    int ret;
    switch (origin)
    {
    case ZLIB_FILEFUNC_SEEK_CUR :
        qiodevice_seek_result = ((QIODevice*)stream)->pos() + offset;
        break;
    case ZLIB_FILEFUNC_SEEK_END :
        qiodevice_seek_result = ((QIODevice*)stream)->size() - offset;
        break;
    case ZLIB_FILEFUNC_SEEK_SET :
        qiodevice_seek_result = offset;
        break;
    default:
        return -1;
    }
    ret = !iodevice->seek(qiodevice_seek_result);
    return ret;
}

int ZCALLBACK qiodevice_close_file_func (
   voidpf opaque,
   voidpf stream)
{
    QIODevice_descriptor *d = reinterpret_cast<QIODevice_descriptor*>(opaque);
    delete d;
    QIODevice *device = reinterpret_cast<QIODevice*>(stream);
#ifdef QUAZIP_QSAVEFILE_BUG_WORKAROUND
    // QSaveFile terribly breaks the is-a idiom:
    // it IS a QIODevice, but it is NOT compatible with it: close() is private
    QSaveFile *file = qobject_cast<QSaveFile*>(device);
    if (file != NULL) {
        // We have to call the ugly commit() instead:
        return file->commit() ? 0 : -1;
    }
#endif
    device->close();
    return 0;
}

int ZCALLBACK qiodevice_fakeclose_file_func (
   voidpf opaque,
   voidpf /*stream*/)
{
    QIODevice_descriptor *d = reinterpret_cast<QIODevice_descriptor*>(opaque);
    delete d;
    return 0;
}

int ZCALLBACK qiodevice_error_file_func (
   voidpf /*opaque UNUSED*/,
   voidpf /*stream UNUSED*/)
{
    // can't check for error due to the QIODevice API limitation
    return 0;
}

void fill_qiodevice_filefunc (
  zlib_filefunc_def* pzlib_filefunc_def)
{
    pzlib_filefunc_def->zopen_file = qiodevice_open_file_func;
    pzlib_filefunc_def->zread_file = qiodevice_read_file_func;
    pzlib_filefunc_def->zwrite_file = qiodevice_write_file_func;
    pzlib_filefunc_def->ztell_file = qiodevice_tell_file_func;
    pzlib_filefunc_def->zseek_file = qiodevice_seek_file_func;
    pzlib_filefunc_def->zclose_file = qiodevice_close_file_func;
    pzlib_filefunc_def->zerror_file = qiodevice_error_file_func;
    pzlib_filefunc_def->opaque = new QIODevice_descriptor;
}

void fill_qiodevice64_filefunc (
  zlib_filefunc64_def* pzlib_filefunc_def)
{
    // Open functions are the same for Qt.
    pzlib_filefunc_def->zopen64_file = qiodevice_open_file_func;
    pzlib_filefunc_def->zread_file = qiodevice_read_file_func;
    pzlib_filefunc_def->zwrite_file = qiodevice_write_file_func;
    pzlib_filefunc_def->ztell64_file = qiodevice64_tell_file_func;
    pzlib_filefunc_def->zseek64_file = qiodevice64_seek_file_func;
    pzlib_filefunc_def->zclose_file = qiodevice_close_file_func;
    pzlib_filefunc_def->zerror_file = qiodevice_error_file_func;
    pzlib_filefunc_def->opaque = new QIODevice_descriptor;
    pzlib_filefunc_def->zfakeclose_file = qiodevice_fakeclose_file_func;
}

void fill_zlib_filefunc64_32_def_from_filefunc32(zlib_filefunc64_32_def* p_filefunc64_32,const zlib_filefunc_def* p_filefunc32)
{
    p_filefunc64_32->zfile_func64.zopen64_file = NULL;
    p_filefunc64_32->zopen32_file = p_filefunc32->zopen_file;
    p_filefunc64_32->zfile_func64.zerror_file = p_filefunc32->zerror_file;
    p_filefunc64_32->zfile_func64.zread_file = p_filefunc32->zread_file;
    p_filefunc64_32->zfile_func64.zwrite_file = p_filefunc32->zwrite_file;
    p_filefunc64_32->zfile_func64.ztell64_file = NULL;
    p_filefunc64_32->zfile_func64.zseek64_file = NULL;
    p_filefunc64_32->zfile_func64.zclose_file = p_filefunc32->zclose_file;
    p_filefunc64_32->zfile_func64.zerror_file = p_filefunc32->zerror_file;
    p_filefunc64_32->zfile_func64.opaque = p_filefunc32->opaque;
    p_filefunc64_32->zfile_func64.zfakeclose_file = NULL;
    p_filefunc64_32->zseek32_file = p_filefunc32->zseek_file;
    p_filefunc64_32->ztell32_file = p_filefunc32->ztell_file;
}
