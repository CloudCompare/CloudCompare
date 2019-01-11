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

#ifndef CC_COLOR_SCALE_SELECTOR_HEADER
#define CC_COLOR_SCALE_SELECTOR_HEADER

//Qt
#include <QFrame>

//qCC_db
#include <ccColorScale.h>

class QComboBox;
class QToolButton;
class ccColorScalesManager;

//! Advanced editor for color scales
/** Combo-box + shortcut to color scale editor
**/
class ccColorScaleSelector : public QFrame
{
	Q_OBJECT

public:

	//! Default constructor
	ccColorScaleSelector(ccColorScalesManager* manager, QWidget* parent, QString defaultButtonIconPath = QString());

	//! Inits selector with the Color Scales Manager
	void init();

	//! Sets selected combo box item (scale) by UUID
	void setSelectedScale(QString uuid);

	//! Returns currently selected color scale
	ccColorScale::Shared getSelectedScale() const;

	//! Returns a given color scale by index
	ccColorScale::Shared getScale(int index) const;

signals:

	//! Signal emitted when a color scale is selected
	void colorScaleSelected(int);

	//! Signal emitted when the user clicks on the 'Spawn Color scale editor' button
	void colorScaleEditorSummoned();

protected:

	//! Color scales manager
	ccColorScalesManager* m_manager;

	//! Color scales combo-box
	QComboBox* m_comboBox;

	//! Spawn color scale editor button
	QToolButton* m_button;
};

#endif //CC_COLOR_SCALE_SELECTOR_HEADER