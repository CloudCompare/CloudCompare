// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: CloudCompare project                               #
// #                                                                        #
// ##########################################################################

#include "ccAboutDialog.h"

// Ui
#include <ui_aboutDlg.h>

// CCAppCommon
#include <ccApplicationBase.h>

ccAboutDialog::ccAboutDialog(QWidget* parent)
    : QDialog(parent)
    , m_ui(std::make_unique<Ui::AboutDialog>())
{
	setAttribute(Qt::WA_DeleteOnClose);

	m_ui->setupUi(this);

	QString compilationInfo;

	compilationInfo = ccApp->versionLongStr(true);
	compilationInfo += QStringLiteral("<br><i>Compiled with");

#if defined(_MSC_VER)
	compilationInfo += QStringLiteral(" MSVC %1 and").arg(_MSC_VER);
#endif

	compilationInfo += QStringLiteral(" Qt %1").arg(QT_VERSION_STR);
	compilationInfo += QStringLiteral("</i>");

	QString htmlText         = m_ui->labelText->text();
	QString enrichedHtmlText = htmlText.arg(compilationInfo);

	m_ui->labelText->setText(enrichedHtmlText);
}

ccAboutDialog::~ccAboutDialog() = default;
