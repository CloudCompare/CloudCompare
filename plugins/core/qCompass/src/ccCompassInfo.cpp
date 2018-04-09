//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
//#                                                                        #
//##########################################################################

#include "ccCompassInfo.h"

ccCompassInfo::ccCompassInfo(QWidget *parent)
	: QDialog(parent)
{
	setFixedWidth(800);
	setFixedHeight(600);

	//setup GUI components
	QTextEdit* l = new QTextEdit(this);
	l->acceptRichText();
	l->setReadOnly(true);

	QPushButton *okButton = new QPushButton(tr("Ok"));
	QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
	buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
	connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept); //done
	QVBoxLayout *lt = new QVBoxLayout;
	lt->addWidget(l);
	lt->addWidget(buttonBox);
	setLayout(lt);

	//load text
	QFile file(":/CC/plugin/qCompass/info.html");
	if (file.open(QIODevice::ReadOnly))
	{
		QTextStream in(&file);
		QString html = in.readAll();
		l->setText(html);
		file.close();
	}
	else
	{
		l->setText("Error loading documentation file....");
	}
}