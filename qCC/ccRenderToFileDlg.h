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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2011                                                              $
//$LastChangedDate:: 2012-02-01 00:15:21 +0100 (mer., 01 févr. 2012)      $
//**************************************************************************
//

#ifndef CC_RENDER_TO_FILE_DLG_HEADER
#define CC_RENDER_TO_FILE_DLG_HEADER

#include <QDialog>

#include <ui_renderToFileDialog.h>

//! Dialog for screen to file rendering
class ccRenderToFileDlg : public QDialog, public Ui::RenderToFileDialog
{
    Q_OBJECT

    public:

		//! Default constructor
        ccRenderToFileDlg(unsigned baseWidth, unsigned baseHeight, QWidget* parent=0);

		//! On dialog acceptance, returns requested zoom
        double getZoom() const;
		//! On dialog acceptance, returns requested output filename
        QString getFilename() const;
		//! On dialog acceptance, returns whether points should be scaled or not
		bool dontScalePoints() const;

    protected slots:

        void chooseFile();
        void updateInfo();
        void saveSettings();

    protected:

        unsigned w;
        unsigned h;

		QString selectedFilter;
        QString currentPath;
        QString filters;
};

#endif

