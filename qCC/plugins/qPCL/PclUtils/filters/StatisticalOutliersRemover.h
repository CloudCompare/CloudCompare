//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#ifndef STATISTICALOUTLIERSREMOVER_H
#define STATISTICALOUTLIERSREMOVER_H

#include <BaseFilter.h>

class ComputeSPINImages;

class StatisticalOutliersRemover : public BaseFilter
{
    Q_OBJECT
public:
    StatisticalOutliersRemover();

protected:
    int compute();
    int openDialog();
    void getParametersFromDialog();
    ComputeSPINImages * m_dialog;
    int m_k;
    float m_std;
};

#endif // STATISTICALOUTLIERREMOVER_H
