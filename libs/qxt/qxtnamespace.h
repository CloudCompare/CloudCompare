/****************************************************************************
 **
 ** Copyright (C) Qxt Foundation. Some rights reserved.
 **
 ** This file is part of the QxtCore module of the Qxt library.
 **
 ** This library is free software; you can redistribute it and/or modify it
 ** under the terms of the Common Public License, version 1.0, as published
 ** by IBM, and/or under the terms of the GNU Lesser General Public License,
 ** version 2.1, as published by the Free Software Foundation.
 **
 ** This file is provided "AS IS", without WARRANTIES OR CONDITIONS OF ANY
 ** KIND, EITHER EXPRESS OR IMPLIED INCLUDING, WITHOUT LIMITATION, ANY
 ** WARRANTIES OR CONDITIONS OF TITLE, NON-INFRINGEMENT, MERCHANTABILITY OR
 ** FITNESS FOR A PARTICULAR PURPOSE.
 **
 ** You should have received a copy of the CPL and the LGPL along with this
 ** file. See the LICENSE file and the cpl1.0.txt/lgpl-2.1.txt files
 ** included with the source distribution for more information.
 ** If you did not receive a copy of the licenses, contact the Qxt Foundation.
 **
 ** <http://libqxt.org>  <foundation@libqxt.org>
 **
 ****************************************************************************/

#ifndef QXTNAMESPACE_H
#define QXTNAMESPACE_H

#include "qxtglobal.h"

#if (defined BUILD_QXT | defined Q_MOC_RUN) && !defined(QXT_DOXYGEN_RUN)
#include <QObject>

class QXT_CORE_EXPORT Qxt  : public QObject
{
    Q_OBJECT
    Q_ENUMS(Rotation)
    Q_ENUMS(DecorationStyle)
    Q_ENUMS(ErrorCode)

public:
#else
namespace Qxt
{
#endif
    enum Rotation
    {
        NoRotation  = 0,
        UpsideDown  = 180,
        Clockwise  = 90,
        CounterClockwise = 270
    };

    enum DecorationStyle
    {
        NoDecoration,
        Buttonlike,
        Menulike
    };

    enum ErrorCode
    {
        NoError,
        UnknownError,
        LogicalError,
        Bug,
        UnexpectedEndOfFunction,
        NotImplemented,
        CodecError,
        NotInitialised,
        EndOfFile,
        FileIOError,
        FormatError,
        DeviceError,
        SDLError,
        InsufficientMemory,
        SeeErrorString,
        UnexpectedNullParameter,
        ClientTimeout,
        SocketIOError,
        ParserError,
        HeaderTooLong,
        Auth,
        Overflow
    };

    enum QxtItemDataRole
    {
        ItemStartTimeRole  = Qt::UserRole + 1,
        ItemDurationRole   = ItemStartTimeRole + 1,
        UserRole           = ItemDurationRole + 23
    };

    enum Timeunit
    {
        Second,
        Minute,
        Hour,
        Day,
        Week,
        Month,
        Year
    };
};

#endif // QXTNAMESPACE_H
