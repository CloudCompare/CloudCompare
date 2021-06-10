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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "PlyOpenDlg.h"

//Qt
#include <QMessageBox>
#include <QStringList>

//qCC_db
#include <ccLog.h>

//System
#include <string.h>
#include <assert.h>

//! Ply dialog loading context
struct PlyLoadingContext
{
	PlyLoadingContext()
		: ignoredProps(0)
		, valid(false)
		, applyAll(false)
	{}
	
	QStringList allProperties;
	std::vector<QString> standardCombosProperties;
	std::vector<QString> sfCombosProperties;
	std::vector<QString> listCombosProperties;
	std::vector<QString> singleCombosProperties;
	int ignoredProps;
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

		m_singleCombos.push_back(texIndexComboBox);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory?! What can we do...
	}

	connect(addSFToolButton,	&QAbstractButton::clicked,	this,	&PlyOpenDlg::addSFComboBox);
	connect(applyButton,		&QAbstractButton::clicked,	this,	&PlyOpenDlg::apply);
	connect(applyAllButton,		&QAbstractButton::clicked,	this,	&PlyOpenDlg::applyAll);
	connect(cancelButton,		&QAbstractButton::clicked,	this,	&QDialog::reject);
	connect(this,				&PlyOpenDlg::fullyAccepted,	this,	&QDialog::accept);
}

void PlyOpenDlg::setDefaultComboItems(const QStringList& stdPropsText)
{
	m_stdPropsText = stdPropsText;
	int stdPropsCount = stdPropsText.count();

	for (QComboBox* combo : m_standardCombos)
	{
		assert(combo);
		combo->addItems(m_stdPropsText);
		combo->setMaxVisibleItems(stdPropsCount);
	}

	for (QComboBox* combo : m_sfCombos)
	{
		assert(combo);
		combo->addItems(m_stdPropsText);
		combo->setMaxVisibleItems(stdPropsCount);
	}
}

void PlyOpenDlg::setListComboItems(const QStringList& listPropsText)
{
	m_listPropsText = listPropsText;
	int listPropsCount = listPropsText.count();

	for (QComboBox* combo : m_listCombos)
	{
		assert(combo);
		combo->addItems(m_listPropsText);
		combo->setMaxVisibleItems(listPropsCount);
	}
}

void PlyOpenDlg::setSingleComboItems(const QStringList& singlePropsText)
{
	m_singlePropsText = singlePropsText;
	int singlePropsCount = singlePropsText.count();

	for (QComboBox* combo : m_singleCombos)
	{
		assert(combo);
		combo->addItems(m_singlePropsText);
		combo->setMaxVisibleItems(singlePropsCount);
	}
}

void PlyOpenDlg::ResetApplyAll()
{
	s_lastContext.applyAll = false;
}

bool PlyOpenDlg::restorePreviousContext(bool& hasAPreviousContext)
{
	hasAPreviousContext = s_lastContext.valid;
	if (!hasAPreviousContext)
		return false;

	int unassignedProps = 0;
	int mismatchProps = 0;
	bool restored = restoreContext(&s_lastContext, unassignedProps, mismatchProps);

	//auto-stop: we can't keep 'apply all' if something has changed
	if (!restored || mismatchProps != 0/* || unassignedProps > 0*/)
	{
		s_lastContext.applyAll = false;
		return false;
	}

	return true;
}

void PlyOpenDlg::saveContext(PlyLoadingContext* context)
{
	if (!context)
	{
		assert(false);
		return;
	}
	context->valid = false;

	//create the list of all properties
	context->allProperties.clear();
	assert(m_standardCombos.front());
	if (m_standardCombos.front())
		for (int i = 1; i < m_standardCombos.front()->count(); ++i) //the first item is always 'NONE'
			context->allProperties.append(m_standardCombos.front()->itemText(i));
	assert(m_listCombos.front());
	if (m_listCombos.front())
		for (int i = 1; i < m_listCombos.front()->count(); ++i) //the first item is always 'NONE'
			context->allProperties.append(m_listCombos.front()->itemText(i));
	assert(m_singleCombos.front());
	if (m_singleCombos.front())
		for (int i = 1; i < m_singleCombos.front()->count(); ++i) //the first item is always 'NONE'
			context->allProperties.append(m_singleCombos.front()->itemText(i));

	//now remember how each combo-box is mapped
	int assignedProps = 0;
	try
	{
		//standard combos
		{
			context->standardCombosProperties.resize(m_standardCombos.size());
			std::fill(context->standardCombosProperties.begin(), context->standardCombosProperties.end(), QString());
			for (size_t i = 0; i < m_standardCombos.size(); ++i)
			{
				if (m_standardCombos[i] && m_standardCombos[i]->currentIndex() > 0) //currentIndex == 0 means 'NONE'!!!
				{
					context->standardCombosProperties[i] = m_standardCombos[i]->currentText();
					++assignedProps;
				}
			}
		}
		//list combos
		{
			context->listCombosProperties.resize(m_listCombos.size());
			std::fill(context->listCombosProperties.begin(), context->listCombosProperties.end(), QString());
			for (size_t i = 0; i < m_listCombos.size(); ++i)
			{
				if (m_listCombos[i] && m_listCombos[i]->currentIndex() > 0)
				{
					context->listCombosProperties[i] = m_listCombos[i]->currentText();
					++assignedProps;
				}
			}
		}
		//single combos
		{
			context->singleCombosProperties.resize(m_singleCombos.size());
			std::fill(context->singleCombosProperties.begin(), context->singleCombosProperties.end(), QString());
			for (size_t i = 0; i < m_singleCombos.size(); ++i)
			{
				if (m_singleCombos[i] && m_singleCombos[i]->currentIndex() > 0)
				{
					context->singleCombosProperties[i] = m_singleCombos[i]->currentText();
					++assignedProps;
				}
			}
		}
		//additional SF combos
		{
			context->sfCombosProperties.clear();
			for (size_t i = 0; i < m_sfCombos.size(); ++i)
			{
				//we only copy the valid ones!
				if (m_sfCombos[i] && m_sfCombos[i]->currentIndex() > 0)
				{
					context->sfCombosProperties.push_back(m_sfCombos[i]->currentText());
					++assignedProps;
				}
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return;
	}

	context->ignoredProps = context->allProperties.size() - assignedProps;
	context->valid = true;
}

bool PlyOpenDlg::restoreContext(PlyLoadingContext* context, int& unassignedProps, int& mismatchProps)
{
	if (!context || !context->valid)
	{
		assert(false);
		return false;
	}

	//first check if all new properties are in the old properties set
	mismatchProps = 0;
	int totalProps = 0;
	{
		assert(m_standardCombos.front());
		if (m_standardCombos.front())
			for (int i = 1; i < m_standardCombos.front()->count(); ++i) //the first item is always 'NONE'
			{
				++totalProps;
				if (!context->allProperties.contains(m_standardCombos.front()->itemText(i)))
					++mismatchProps;
			}
		assert(m_listCombos.front());
		if (m_listCombos.front())
			for (int i = 1; i < m_listCombos.front()->count(); ++i) //the first item is always 'NONE'
			{
				++totalProps;
				if (!context->allProperties.contains(m_listCombos.front()->itemText(i)))
					++mismatchProps;
			}
		assert(m_singleCombos.front());
		if (m_singleCombos.front())
			for (int i = 1; i < m_singleCombos.front()->count(); ++i) //the first item is always 'NONE'
			{
				++totalProps;
				if (!context->allProperties.contains(m_singleCombos.front()->itemText(i)))
					++mismatchProps;
			}
	}

	int assignedEntries = 0;

	//standard combos
	assert(m_standardCombos.size() == context->standardCombosProperties.size());
	{
		for (size_t i = 0; i < m_standardCombos.size(); ++i)
		{
			if (m_standardCombos[i])
			{
				m_standardCombos[i]->setCurrentIndex(0);
				//if a specific property was defined for this field
				if (!context->standardCombosProperties[i].isNull())
				{
					//try to find it in the new property list!
					int idx = m_standardCombos[i]->findText(context->standardCombosProperties[i]);
					if (idx >= 0)
					{
						++assignedEntries;
						m_standardCombos[i]->setCurrentIndex(idx);
					}
				}
			}
		}
	}

	//list combos
	assert(m_listCombos.size() == context->listCombosProperties.size());
	{
		for (size_t i = 0; i < m_listCombos.size(); ++i)
		{
			if (m_listCombos[i])
			{
				m_listCombos[i]->setCurrentIndex(0);
				//if a specific property was defined for this field
				if (!context->listCombosProperties[i].isNull())
				{
					//try to find it in the new property list!
					int idx = m_listCombos[i]->findText(context->listCombosProperties[i]);
					if (idx >= 0)
					{
						++assignedEntries;
						m_listCombos[i]->setCurrentIndex(idx);
					}
				}
			}
		}
	}

	//single combox
	assert(m_singleCombos.size() == context->singleCombosProperties.size());
	{
		for (size_t i = 0; i < m_singleCombos.size(); ++i)
		{
			if (m_singleCombos[i])
			{
				m_singleCombos[i]->setCurrentIndex(0);
				//if a specific property was defined for this field
				if (!context->singleCombosProperties[i].isNull())
				{
					//try to find it in the new property list!
					int idx = m_singleCombos[i]->findText(context->singleCombosProperties[i]);
					if (idx >= 0)
					{
						++assignedEntries;
						m_singleCombos[i]->setCurrentIndex(idx);
					}
				}
			}
		}
	}
	
	//additional SF combos
	assert(m_sfCombos.size() == 1);
	{
		m_sfCombos.front()->setCurrentIndex(0);
		bool firstSF = true;
		for (size_t i = 0; i < context->sfCombosProperties.size(); ++i)
		{
			//try to find it in the new property list!
			int idx = m_sfCombos.front()->findText(context->sfCombosProperties[i]);
			if (idx >= 0)
			{
				++assignedEntries;
				//we use the default sf combo-box by default
				if (firstSF)
					m_sfCombos.front()->setCurrentIndex(idx);
				else
					addSFComboBox(idx);
				firstSF = false;
			}
		}
	}

	assert(assignedEntries <= totalProps);
	unassignedProps = totalProps - assignedEntries;

	return true;
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
			QMessageBox::warning(nullptr, "Error", "At least two vertex coordinates (X,Y,Z) must be defined!");
		return false;
	}

	//we must ensure that no property is assigned to more than one field
	int n = m_stdPropsText.size();
	int p = m_listPropsText.size();
	int q = m_singlePropsText.size();

	assert(n + p + q >= 2);
	std::vector<int> assignedIndexCount(n + p + q, 0);

	for (size_t i = 0; i < m_standardCombos.size(); ++i)
		++assignedIndexCount[m_standardCombos[i]->currentIndex()];
	for (size_t j = 0; j < m_listCombos.size(); ++j)
		++assignedIndexCount[m_listCombos[j]->currentIndex() > 0 ? n + m_listCombos[j]->currentIndex() : 0];
	for (size_t k = 0; k < m_singleCombos.size(); ++k)
		++assignedIndexCount[m_singleCombos[k]->currentIndex() > 0 ? n + p + m_singleCombos[k]->currentIndex() : 0];
	for (size_t l = 0; l < m_sfCombos.size(); ++l)
		++assignedIndexCount[m_sfCombos[l]->currentIndex()];

	for (int i = 1; i < n + p + q; ++i)
	{
		if (assignedIndexCount[i] > 1)
		{
			if (displayErrors)
				QMessageBox::warning(nullptr, "Error", QString("Can't assign same property to multiple fields! (%1)").arg(xComboBox->itemText(i)));
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
