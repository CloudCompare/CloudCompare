#pragma once

//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                   COPYRIGHT: Thomas Montaigu                           #
//#                                                                        #
//##########################################################################

#include "LasDetails.h"
#include "LasExtraScalarField.h"

#include <laszip/laszip_api.h>

#include <QMetaType>
#include <QString>

struct LasVlr
{
    LasVlr() = default;

    explicit LasVlr(const laszip_header &header);

    LasVlr(const LasVlr &rhs);
    LasVlr &operator=(LasVlr rhs);
    static void Swap(LasVlr &lhs, LasVlr &rhs) noexcept;

    virtual ~LasVlr() noexcept;

    laszip_U32 numVlrs{0};
    laszip_vlr_struct *vlrs{nullptr};
    std::vector<LasExtraScalarField> extraScalarFields{};

    QString toString() const
    {
        return QString("Vlrs: %1").arg(numVlrs);
    }
};

Q_DECLARE_METATYPE(LasVlr);
