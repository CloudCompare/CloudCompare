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
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "ccSelectChildrenDlg.h"

#include "ui_selectChildrenDlg.h"

static QString       s_lastName;
static bool          s_lastNameState       = false;
static CC_CLASS_ENUM s_lastType            = CC_TYPES::POINT_CLOUD;
static bool          s_lastTypeState       = true;
static bool          s_lastTypeStrictState = true;
static bool          s_lastUseRegex        = true;

ccSelectChildrenDlg::ccSelectChildrenDlg(QWidget* parent /*=nullptr*/)
    : QDialog(parent, Qt::Tool)
    , m_ui(std::make_unique<Ui::SelectChildrenDialog>())
{
	m_ui->setupUi(this);

	m_ui->typeCheckBox->setChecked(s_lastTypeState);
	m_ui->typeStrictCheckBox->setChecked(s_lastTypeStrictState);
	m_ui->nameCheckBox->setChecked(s_lastNameState);
	m_ui->nameLineEdit->setText(s_lastName);
	m_ui->checkBoxRegex->setChecked(s_lastUseRegex);

	connect(m_ui->buttonBox, &QDialogButtonBox::accepted, this, &ccSelectChildrenDlg::onAccept);
}

ccSelectChildrenDlg::~ccSelectChildrenDlg() = default;

void ccSelectChildrenDlg::addType(QString typeName, CC_CLASS_ENUM type)
{
	m_ui->typeComboBox->addItem(typeName, QVariant::fromValue<qint64>(type));

	// auto select last selected type
	if (type == s_lastType)
	{
		m_ui->typeComboBox->setCurrentIndex(m_ui->typeComboBox->count() - 1);
	}
}

void ccSelectChildrenDlg::onAccept()
{
	s_lastNameState       = m_ui->nameCheckBox->isChecked();
	s_lastName            = m_ui->nameLineEdit->text();
	s_lastTypeState       = m_ui->typeCheckBox->isChecked();
	s_lastTypeStrictState = m_ui->typeCheckBox->isChecked();
	s_lastType            = getSelectedType();
	s_lastUseRegex        = getNameIsRegex();
}

CC_CLASS_ENUM ccSelectChildrenDlg::getSelectedType()
{
	if (!m_ui->typeCheckBox->isChecked())
	{
		return CC_TYPES::HIERARCHY_OBJECT;
	}

	int currentIndex = m_ui->typeComboBox->currentIndex();
	return static_cast<CC_CLASS_ENUM>(m_ui->typeComboBox->itemData(currentIndex).value<qint64>());
}

QString ccSelectChildrenDlg::getSelectedName()
{
	if (!m_ui->nameCheckBox->isChecked())
	{
		return QString();
	}

	return m_ui->nameLineEdit->text();
}

bool ccSelectChildrenDlg::getStrictMatchState() const
{
	return m_ui->typeStrictCheckBox->isChecked();
}

bool ccSelectChildrenDlg::getTypeIsUsed() const
{
	return m_ui->typeCheckBox->isChecked();
}

bool ccSelectChildrenDlg::getNameIsRegex() const
{
	return m_ui->checkBoxRegex->isChecked();
}

bool ccSelectChildrenDlg::getNameMatchIsUsed() const
{
	return m_ui->nameCheckBox->isChecked();
}
