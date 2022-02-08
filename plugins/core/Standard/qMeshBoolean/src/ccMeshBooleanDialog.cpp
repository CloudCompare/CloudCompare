//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qMeshBoolean                    #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "ccMeshBooleanDialog.h"

ccMeshBooleanDialog::ccMeshBooleanDialog(QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, Ui::MeshBooleanDialog()
	, m_selectedOperation(UNION)
	, m_isSwapped(false)
{
	setupUi(this);

	connect(unionPushButton,	SIGNAL(clicked()), this, SLOT(unionSelected()));
	connect(interPushButton,	SIGNAL(clicked()), this, SLOT(intersectSelected()));
	connect(diffPushButton,		SIGNAL(clicked()), this, SLOT(diffSelected()));
	connect(symDiffPushButton,	SIGNAL(clicked()), this, SLOT(symDiffSelected()));
	connect(swapToolButton,		SIGNAL(clicked()), this, SLOT(swap()));
}

void ccMeshBooleanDialog::setNames(const QString& A, const QString& B)
{
	meshALineEdit->setText(A);
	meshBLineEdit->setText(B);
}

void ccMeshBooleanDialog::unionSelected()
{
	m_selectedOperation = UNION;
	accept();
}

void ccMeshBooleanDialog::intersectSelected()
{
	m_selectedOperation = INTERSECT;
	accept();
}

void ccMeshBooleanDialog::diffSelected()
{
	m_selectedOperation = DIFF;
	accept();
}

void ccMeshBooleanDialog::symDiffSelected()
{
	m_selectedOperation = SYM_DIFF;
	accept();
}

void ccMeshBooleanDialog::swap()
{
	m_isSwapped = !m_isSwapped;

	QString A = meshALineEdit->text();
	QString B = meshBLineEdit->text();
	
	setNames(B,A);
}
