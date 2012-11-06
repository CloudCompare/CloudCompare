//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#ifndef CC_ASK_ONE_STRING_DLG_HEADER
#define CC_ASK_ONE_STRING_DLG_HEADER

#include <ui_askOneStringDlg.h>

class ccAskOneStringDlg : public QDialog, public Ui::AskOneStringDlg
{
public:
    ccAskOneStringDlg(const QString& label,
                            const QString& initVal = QString(),
								const QString& windowTitle = QString(),
									QWidget* parent=0);

	const QString result() {return lineEdit->text();}
};

#endif
