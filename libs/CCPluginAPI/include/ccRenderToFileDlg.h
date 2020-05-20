#pragma once
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

#include "CCPluginAPI.h"

#include <QDialog>

namespace Ui
{
	class RenderToFileDialog;	
}

//! Dialog for screen to file rendering
class CCPLUGIN_LIB_API ccRenderToFileDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	ccRenderToFileDlg(unsigned baseWidth, unsigned baseHeight, QWidget* parent = 0);

	~ccRenderToFileDlg() override;
	
	//! Disable and hide the scale and overlay checkboxes
	void hideOptions();
	
	//! On dialog acceptance, returns requested zoom
	float getZoom() const;
	//! On dialog acceptance, returns requested output filename
	QString getFilename() const;
	//! On dialog acceptance, returns whether points should be scaled or not
	bool dontScalePoints() const;
	//! Whether overlay items should be rendered
	bool renderOverlayItems() const;

private:

	void chooseFile();
	void updateInfo();
	void saveSettings();

	unsigned w;
	unsigned h;

	QString selectedFilter;
	QString currentPath;
	QString filters;

	Ui::RenderToFileDialog* m_ui;
};
