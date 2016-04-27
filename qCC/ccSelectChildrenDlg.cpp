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

#include "ccSelectChildrenDlg.h"

static QString s_lastName;
static bool s_lastNameState = false;
static CC_CLASS_ENUM s_lastType = CC_TYPES::POINT_CLOUD;
static bool s_lastTypeState = true;
static bool s_lastTypeStrictState = true;

ccSelectChildrenDlg::ccSelectChildrenDlg(QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::SelectChildrenDialog()
{
	setupUi(this);

	typeCheckBox->setChecked(s_lastTypeState);
	typeStrictCheckBox->setChecked(s_lastTypeStrictState);
	nameCheckBox->setChecked(s_lastNameState);
	nameLineEdit->setText(s_lastName);

	connect(buttonBox, SIGNAL(accepted()), this, SLOT(onAccept()));
}

void ccSelectChildrenDlg::addType(QString typeName, CC_CLASS_ENUM type)
{
	typeComboBox->addItem(typeName,QVariant::fromValue<qint64>(type));

	//auto select last selected type
	if (type == s_lastType)
		typeComboBox->setCurrentIndex(typeComboBox->count()-1);
}

void ccSelectChildrenDlg::onAccept()
{
	s_lastNameState = nameCheckBox->isChecked();
	s_lastName = nameLineEdit->text();
	s_lastTypeState = typeCheckBox->isChecked();
	s_lastTypeStrictState = typeCheckBox->isChecked();
	s_lastType = getSelectedType();
}

CC_CLASS_ENUM ccSelectChildrenDlg::getSelectedType()
{
	if (!typeCheckBox->isChecked())
		return CC_TYPES::HIERARCHY_OBJECT;

	int currentIndex = typeComboBox->currentIndex();
	return static_cast<CC_CLASS_ENUM>(typeComboBox->itemData(currentIndex).value<qint64>());
}

QString ccSelectChildrenDlg::getSelectedName()
{
	if (!nameCheckBox->isChecked())
		return QString();

	return nameLineEdit->text();
}

bool ccSelectChildrenDlg::getStrictMatchState()
{
    return typeStrictCheckBox->isChecked();
}

bool ccSelectChildrenDlg::getNameIsRegex() const
{

    return checkBoxRegex->isChecked();
}
