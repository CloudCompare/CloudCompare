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

#include "LasExtraScalarField.h"
#include "ui_extra_scarlar_field_card.h"


class LasExtraScalarFieldCard : public QWidget, public Ui::ExtraScalarFieldCard
{
    Q_OBJECT

  public:
    explicit LasExtraScalarFieldCard(QWidget *parent = nullptr);

    void fillFrom(const LasExtraScalarField &field);

    void reset();

    LasExtraScalarField::DataType dataType() const;

     bool fillField(LasExtraScalarField &field, const ccPointCloud &pointCloud) const;
};