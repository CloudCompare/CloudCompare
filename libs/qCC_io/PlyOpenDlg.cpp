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

#include "PlyOpenDlg.h"

//Qt
#include <QMessageBox>

//System
#include <string.h>
#include <assert.h>

PlyOpenDlg::PlyOpenDlg(QWidget* parent) : QDialog(parent), Ui::PlyOpenDlg()
{
	setupUi(this);

	try
	{
		m_standardCombos.push_back(xComboBox);
		m_standardCombos.push_back(yComboBox);
		m_standardCombos.push_back(zComboBox);
		m_standardCombos.push_back(rComboBox);
		m_standardCombos.push_back(gComboBox);
		m_standardCombos.push_back(bComboBox);
		m_standardCombos.push_back(iComboBox);
		m_standardCombos.push_back(nxComboBox);
		m_standardCombos.push_back(nyComboBox);
		m_standardCombos.push_back(nzComboBox);

		m_sfCombos.push_back(sfComboBox);

		m_listCombos.push_back(facesComboBox);
		m_listCombos.push_back(textCoordsComboBox);
	}
	catch(std::bad_alloc)
	{
	}

	connect(addSFToolButton,	SIGNAL(clicked()),			this,	SLOT(addSFComboBox()));
	connect(buttonBox,			SIGNAL(accepted()),			this,	SLOT(testBeforeAccept()));
	connect(this,				SIGNAL(fullyAccepted()),	this,	SLOT(accept()));
}

void PlyOpenDlg::setDefaultComboItems(const QStringList& stdPropsText)
{
	m_stdPropsText = stdPropsText;
	int stdPropsCount = stdPropsText.count();

	for (size_t i=0; i<m_standardCombos.size(); ++i)
	{
		assert(m_standardCombos[i]);
		m_standardCombos[i]->addItems(m_stdPropsText);
		m_standardCombos[i]->setMaxVisibleItems(stdPropsCount);
	}

	for (size_t j=0; j<m_sfCombos.size(); ++j)
	{
		assert(m_sfCombos[j]);
		m_sfCombos[j]->addItems(m_stdPropsText);
		m_sfCombos[j]->setMaxVisibleItems(stdPropsCount);
	}
}

void PlyOpenDlg::setListComboItems(const QStringList& listPropsText)
{
	m_listPropsText = listPropsText;
	int listPropsCount = listPropsText.count();

	for (size_t i=0; i<m_listCombos.size(); ++i)
	{
		assert(m_listCombos[i]);
		m_listCombos[i]->addItems(m_listPropsText);
		m_listCombos[i]->setMaxVisibleItems(listPropsCount);
	}
}

void PlyOpenDlg::testBeforeAccept()
{
	//we need at least two coordinates per point (i.e. 2D)
	int zeroCoord = 0;
	if (xComboBox->currentIndex() == 0) ++zeroCoord;
	if (yComboBox->currentIndex() == 0) ++zeroCoord;
	if (zComboBox->currentIndex() == 0) ++zeroCoord;

	if (zeroCoord > 1)
	{
		QMessageBox::warning(0, "Error", "At least two vertex coordinates (X,Y,Z) must be defined!");
		return;
	}

	//we must ensure that no property is assigned to more than one field
	int n = m_stdPropsText.size();
	int p = m_listPropsText.size();

	assert(n+p >= 2);
	std::vector<int> assignedIndexCount(n+p,0);

	for (size_t i=0; i<m_standardCombos.size(); ++i)
		++assignedIndexCount[m_standardCombos[i]->currentIndex()];
	for (size_t j=0; j<m_listCombos.size(); ++j)
		++assignedIndexCount[m_listCombos[j]->currentIndex() > 0 ? n+m_listCombos[j]->currentIndex() : 0];
	for (size_t k=0; k<m_sfCombos.size(); ++k)
		++assignedIndexCount[m_sfCombos[k]->currentIndex()];

	bool isValid = true;
	{
		for (int i=1; i<n+p; ++i)
		{
			if (assignedIndexCount[i] > 1)
			{
				isValid = false;
				QMessageBox::warning(0, "Error", QString("Can't assign same property to multiple fields! (%1)").arg(xComboBox->itemText(i)));
				break;
			}
		}
	}

	if (isValid)
		emit fullyAccepted();
}

void PlyOpenDlg::addSFComboBox(int selectedIndex/*=0*/)
{
	//create a new combo-box
	m_sfCombos.push_back(new QComboBox());
	formLayout->addRow(QString("Scalar #%1").arg(m_sfCombos.size()), m_sfCombos.back());

	//fill it with default items
	m_sfCombos.back()->addItems(m_stdPropsText);
	m_sfCombos.back()->setMaxVisibleItems(m_stdPropsText.size());
	m_sfCombos.back()->setCurrentIndex(selectedIndex);
}
