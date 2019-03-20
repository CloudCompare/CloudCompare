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

#include <QFile>
#include <QTextEdit>
#include <QTextStream>
#include <QVBoxLayout>

#include "ccCompassInfo.h"

ccCompassInfo::ccCompassInfo(QWidget *parent)
	: QDialog(parent)
{
	setFixedSize(800, 600);

	//setup GUI components
	QTextEdit* l = new QTextEdit;
	l->acceptRichText();
	l->setReadOnly(true);

	QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok, Qt::Horizontal);
	connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
	
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
	}
	else
	{
		l->setText("Error loading documentation file...");
	}
}
