//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "ccCommon.h"

#include "ccAboutDialog.h"

#include "ui_aboutDlg.h"


ccAboutDialog::ccAboutDialog( QWidget *parent )
	: QDialog( parent )
	, mUI( new Ui::AboutDialog )
{
	setAttribute( Qt::WA_DeleteOnClose );
	
	mUI->setupUi(this);
	
	QString compilationInfo;

	compilationInfo = ccCommon::GetCCVersion();
	compilationInfo += QString("<br><i>Compiled with");

#if defined(_MSC_VER)
	compilationInfo += QString(" MSVC %1 and").arg(_MSC_VER);
#endif

	compilationInfo += QString(" Qt %1").arg(QT_VERSION_STR);
	compilationInfo += QString("</i>");
	
	QString htmlText = mUI->textEdit->toHtml();
	QString enrichedHtmlText = htmlText.arg(compilationInfo);
	
	mUI->textEdit->setHtml(enrichedHtmlText);
}

ccAboutDialog::~ccAboutDialog()
{	
	delete mUI;
}
