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

//qCC_db
#include <ccLog.h>

//System
#include <string.h>
#include <assert.h>

//! Ply dialog loading context
struct PlyLoadingContext
{
	PlyLoadingContext() : valid(false), applyAll(false) {}
	
	std::vector<QString> standardCombosProperties;
	std::vector<QString> sfCombosProperties;
	std::vector<QString> listCombosProperties;
	bool valid;
	bool applyAll;
};
//! Last loading context
static PlyLoadingContext s_lastContext;

PlyOpenDlg::PlyOpenDlg(QWidget* parent)
	: QDialog(parent)
	, Ui::PlyOpenDlg()
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
		//not enough memory?! What can we do...
	}

	connect(addSFToolButton,	SIGNAL(clicked()),			this,	SLOT(addSFComboBox()));
	connect(applyButton,		SIGNAL(clicked()),			this,	SLOT(apply()));
	connect(applyAllButton,		SIGNAL(clicked()),			this,	SLOT(applyAll()));
	connect(cancelButton,		SIGNAL(clicked()),			this,	SLOT(reject()));
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

int PlyOpenDlg::restorePreviousContext()
{
	if (!s_lastContext.valid)
		return -1;

	unsigned missingProps = restoreContext(&s_lastContext);

	//auto-stop: we can't keep 'apply all' if something has changed
	if (missingProps != 0)
		s_lastContext.applyAll = false;

	return static_cast<int>(missingProps);
}

void PlyOpenDlg::saveContext(PlyLoadingContext* context)
{
	if (!context)
	{
		assert(false);
		return;
	}
	context->valid = false;

	//currentIndex == 0 means 'NONE'!!!
	try
	{
		//standard combos
		{
			context->standardCombosProperties.resize(m_standardCombos.size());
			for (size_t i=0; i<m_standardCombos.size(); ++i)
				context->standardCombosProperties[i] = (m_standardCombos[i] && m_standardCombos[i]->currentIndex() > 0 ? m_standardCombos[i]->currentText() : QString());
		}
		//list combos
		{
			context->listCombosProperties.resize(m_listCombos.size());
			for (size_t i=0; i<m_listCombos.size(); ++i)
				context->listCombosProperties[i] = (m_listCombos[i] && m_listCombos[i]->currentIndex() > 0 ? m_listCombos[i]->currentText() : QString());
		}
		//additional SF combos
		{
			context->sfCombosProperties.clear();
			for (size_t i=0; i<m_sfCombos.size(); ++i)
			{
				//we only copy the valid ones!
				if (m_sfCombos[i] && m_sfCombos[i]->currentIndex() > 0)
					context->sfCombosProperties.push_back(m_sfCombos[i]->currentText());
			}
		}
	}
	catch(std::bad_alloc)
	{
		//not enough memory
		return;
	}

	context->valid = true;
}

unsigned PlyOpenDlg::restoreContext(PlyLoadingContext* context)
{
	if (!context || !context->valid)
	{
		assert(false);
		return false;
	}

	//currentIndex == 0 means 'NONE'!!!
	unsigned missingEntries = 0;

	//standard combos
	assert(m_standardCombos.size() == context->standardCombosProperties.size());
	{
		for (size_t i=0; i<m_standardCombos.size(); ++i)
			if (m_standardCombos[i])
			{
				m_standardCombos[i]->setCurrentIndex(0);
				//if a specific property was defined for this field
				if (!context->standardCombosProperties[i].isNull())
				{
					//try to find it in the new property list!
					int idx = m_standardCombos[i]->findText(context->standardCombosProperties[i]);
					if (idx < 0)
						++missingEntries;
					m_standardCombos[i]->setCurrentIndex(idx);
				}
			}
	}
	
	//list combos
	assert(m_listCombos.size() == context->listCombosProperties.size());
	{
		for (size_t i=0; i<m_listCombos.size(); ++i)
			if (m_listCombos[i])
			{
				m_listCombos[i]->setCurrentIndex(0);
				//if a specific property was defined for this field
				if (!context->listCombosProperties[i].isNull())
				{
					//try to find it in the new property list!
					int idx = m_listCombos[i]->findText(context->listCombosProperties[i]);
					if (idx < 0)
						++missingEntries;
					m_listCombos[i]->setCurrentIndex(idx);
				}
			}

	}
	
	//additional SF combos
	assert(m_sfCombos.size() == 1);
	{
		m_sfCombos.front()->setCurrentIndex(0);
		bool firstSF = true;
		for (size_t i=0; i<context->sfCombosProperties.size(); ++i)
		{
			//try to find it in the new property list!
			int idx = m_sfCombos.front()->findText(context->sfCombosProperties[i]);
			if (idx < 0)
				++missingEntries;
			else
			{
				//we use the default sf combo-box by default
				if (firstSF)
					m_sfCombos.front()->setCurrentIndex(idx);
				else
					addSFComboBox(idx);
				firstSF = false;
			}
		}
	}

	return missingEntries;
}

void PlyOpenDlg::apply()
{
	if (isValid())
	{
		saveContext(&s_lastContext);
		s_lastContext.applyAll = false;
		emit fullyAccepted();
	}
}

void PlyOpenDlg::applyAll()
{
	if (isValid())
	{
		saveContext(&s_lastContext);
		s_lastContext.applyAll = true;
		emit fullyAccepted();
	}
}

bool PlyOpenDlg::isValid(bool displayErrors/*=true*/) const
{
	//we need at least two coordinates per point (i.e. 2D)
	int zeroCoord = 0;
	if (xComboBox->currentIndex() == 0) ++zeroCoord;
	if (yComboBox->currentIndex() == 0) ++zeroCoord;
	if (zComboBox->currentIndex() == 0) ++zeroCoord;

	if (zeroCoord > 1)
	{
		if (displayErrors)
			QMessageBox::warning(0, "Error", "At least two vertex coordinates (X,Y,Z) must be defined!");
		return false;
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

	for (int i=1; i<n+p; ++i)
	{
		if (assignedIndexCount[i] > 1)
		{
			if (displayErrors)
				QMessageBox::warning(0, "Error", QString("Can't assign same property to multiple fields! (%1)").arg(xComboBox->itemText(i)));
			return false;
		}
	}

	return true;
}

bool PlyOpenDlg::canBeSkipped() const
{
	return s_lastContext.valid && s_lastContext.applyAll && isValid(false);
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
