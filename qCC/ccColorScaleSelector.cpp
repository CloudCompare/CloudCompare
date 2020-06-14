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

#include "ccColorScaleSelector.h"

//Qt
#include <QHBoxLayout>
#include <QComboBox>
#include <QToolButton>

//Local
#include "ccColorScalesManager.h"

ccColorScaleSelector::ccColorScaleSelector(ccColorScalesManager* manager, QWidget* parent, QString defaultButtonIconPath/*=QString()*/)
	: QFrame(parent)
	, m_manager(manager)
	, m_comboBox(new QComboBox())
	, m_button(new QToolButton())
{
	assert(m_manager);
	
	setLayout(new QHBoxLayout());
	layout()->setContentsMargins(0, 0, 0, 0);
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
		m_comboBox->disconnect(this);

		m_comboBox->clear();
		//add all available color scales
		assert(m_manager);

		//sort the scales by their name
		//DGM: See doc about qSort --> "An alternative to using qSort() is to put the items to sort in a QMap, using the sort key as the QMap key."
		QMap<QString, QString> scales;
		for (ccColorScalesManager::ScalesMap::const_iterator it = m_manager->map().constBegin(); it != m_manager->map().constEnd(); ++it)
		{
			scales.insert((*it)->getName(), (*it)->getUuid());
		}

		for (QMap<QString, QString>::const_iterator scale = scales.constBegin(); scale != scales.constEnd(); ++scale)
		{
			m_comboBox->addItem(scale.key(), scale.value());
		}

		connect(m_comboBox, SIGNAL(activated(int)), this, SIGNAL(colorScaleSelected(int)));
	}
	//advanced tool button
	if (m_button)
	{
		m_button->disconnect(this);
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
		return ccColorScale::Shared(nullptr);

	//get UUID associated to the combo-box item
	QString UUID = m_comboBox->itemData(index).toString();

	return m_manager ? m_manager->getScale(UUID) : ccColorScale::Shared(nullptr);
}

void ccColorScaleSelector::setSelectedScale(QString uuid)
{
	if (!m_comboBox)
		return;

	//search right index by UUID
	int pos = m_comboBox->findData(uuid);
	if (pos < 0)
		return;
	m_comboBox->setCurrentIndex(pos);

	emit colorScaleSelected(pos);
}
