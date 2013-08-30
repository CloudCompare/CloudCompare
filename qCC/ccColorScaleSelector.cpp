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

#include "ccColorScaleSelector.h"

//Qt
#include <QHBoxLayout>
#include <QComboBox>
#include <QToolButton>

//Local
#include "ccColorScalesManager.h"

ccColorScaleSelector::ccColorScaleSelector(QWidget* parent, QString defaultButtonIconPath/*=QString()*/)
	: QFrame(parent)
	, m_comboBox(new QComboBox())
	, m_button(new QToolButton())
{
	setLayout(new QHBoxLayout());
	layout()->setContentsMargins(0,0,0,0);
	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);

	//combox box
	if (m_comboBox)
	{
		layout()->addWidget(m_comboBox);
	}

	//tool button
	if (m_button)
	{
		m_button->setIcon(QIcon(defaultButtonIconPath));
		layout()->addWidget(m_button);
	}
}

void ccColorScaleSelector::init()
{
	//fill combox box
	if (m_comboBox)
	{
		m_comboBox->blockSignals(true);
		m_comboBox->clear();
		//add all available color scales
		ccColorScalesManager* csManager = ccColorScalesManager::GetUniqueInstance();
		assert(csManager);
		for (ccColorScalesManager::ScalesMap::const_iterator it = csManager->map().begin(); it != csManager->map().end(); ++it)
			m_comboBox->addItem((*it)->getName(),(*it)->getUuid());
		m_comboBox->blockSignals(false);

		connect(m_comboBox, SIGNAL(activated(int)), this, SIGNAL(colorScaleSelected(int)));
	}
	//advanced tool button
	if (m_button)
	{
		connect(m_button, SIGNAL(clicked()), this, SIGNAL(colorScaleEditorSummoned()));
	}
}

ccColorScale::Shared ccColorScaleSelector::getSelectedScale() const
{
	return getScale(m_comboBox ? m_comboBox->currentIndex() : -1);
}

ccColorScale::Shared ccColorScaleSelector::getScale(int index) const
{
	if (!m_comboBox || index < 0 || index >= m_comboBox->count())
		return ccColorScale::Shared(0);

	//get UUID associated to the combo-box item
	QString UUID = m_comboBox->itemData(index).toString();

	return ccColorScalesManager::GetUniqueInstance()->getScale(UUID);
}

void ccColorScaleSelector::setSelectedScale(QString uuid)
{
	if (!m_comboBox)
		return;

	//search right index by UUID
	int pos = m_comboBox->findData(uuid);
	m_comboBox->setCurrentIndex(pos);
}
