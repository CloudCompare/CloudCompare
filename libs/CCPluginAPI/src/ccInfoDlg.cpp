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
// #                    COPYRIGHT: CloudCompare project                     #
// #                                                                        #
// ##########################################################################

#include "../include/ccInfoDlg.h"

#include "ui_infoDlg.h"

ccInfoDlg::ccInfoDlg(QWidget* parent)
    : QDialog(parent)
    , m_ui(new Ui::InfoDialog)
{
	m_ui->setupUi(this);
}

ccInfoDlg::~ccInfoDlg()
{
	if (m_ui)
	{
		delete m_ui;
		m_ui = nullptr;
	}
}

void ccInfoDlg::showText(const QString& text)
{
	m_ui->textEdit->setText(text);
}
